/** @file 一次用户启动意图下的有界顺序执行；失败不重发，跨连接或启动号不续跑。 */
import {remoteDraftError,remoteDefersFeedback,remoteTerminalConfirmed,remoteHomeError,describeRemoteTerminal,type RemoteDraft,type RemoteOperation,type RemoteReceipt} from './remoteControl';
import {monitorRemoteTransaction} from './remoteMonitor';

/** 可注入的传输和连接生命周期，测试不连接串口。 */
export interface RemoteWorkflowOptions {
  request:(body:Record<string,unknown>)=>Promise<RemoteReceipt>;
  isCurrent:()=>boolean;
  wait:()=>Promise<void>;
  observe:(step:string,receipt:RemoteReceipt)=>void;
}
/** 未知事务编号保留给只读恢复；不得据此重发动作。 */
export interface RemoteWorkflowResult {result:string;detail:string;transactionId:number;operation:RemoteOperation|null}

/** 冻结参数后依次完成原板端流程，每一步新取能力并等待原事务成功。 */
export async function runRemoteWorkflow(input:RemoteDraft,options:RemoteWorkflowOptions):Promise<RemoteWorkflowResult>{
  const draft={...input};
  let pendingOperation:RemoteOperation|null=null;
  const invalid=remoteDraftError(draft);
  const done=(result:string,detail:string,transactionId=0):RemoteWorkflowResult=>({result,detail,transactionId,operation:pendingOperation});
  if(invalid)return done('rejected',invalid);
  const steps:RemoteOperation[]=draft.operation==='hold'?['preflight','configure','acquire','hold','release']:
    ['pos','pos_load','home','mode_mit','mode_pos'].includes(draft.operation)?['acquire',draft.operation,'release']:[draft.operation];
  const desiredMode=['hold','configure','mode_mit'].includes(draft.operation)?'1':
    ['pos','pos_load','home','mode_pos'].includes(draft.operation)?'2':null;
  const explicitMode=draft.operation==='mode_mit'||draft.operation==='mode_pos';
  let boot='',pendingId=0,modePreparation=explicitMode;
  try{
    for(let stepIndex=0;stepIndex<steps.length;stepIndex++){
      let operation=steps[stepIndex];
      if(!options.isCurrent())return done('cancelled','连接或页面已变化，后续步骤已取消',pendingId);
      const caps=await options.request({operation:'caps'});
      if(!options.isCurrent())return done('cancelled','连接已变化，未提交下一步',pendingId);
      const fields=caps.fields;
      if(caps.result!=='capability'||!/^\d+$/.test(fields.boot??'')||Number(fields.boot)<=0||
        !Number.isSafeInteger(Number(fields.token))||Number(fields.token)<=0||fields.active_id!=='0')
        return done('rejected','设备能力不完整或已有事务；未提交下一步');
      if(boot&&boot!==fields.boot)return done('rejected','设备已经重启，启动意图失效');
      boot=fields.boot;
      if(draft.operation==='home'){
        const error=remoteHomeError(draft,fields);
        if(error)return done('rejected',error);
      }
      /* Validate the final intent before acquiring ownership or changing modes on old firmware. */
      if(stepIndex===0){
        if(['hold','configure'].includes(draft.operation)&&fields.static!=='1')return done('rejected','设备未开放静载能力');
        if(draft.operation==='pos'||draft.operation==='pos_load'){
          const loaded=draft.operation==='pos_load',limit=Number(fields[loaded?'load_delta':'max_delta']),speed=Number(fields[loaded?'load_speed':'max_speed']);
          if(fields[loaded?'load':'pos']!=='1'||!Number.isFinite(limit)||limit<=0||!Number.isFinite(speed)||speed<=0||
            Math.abs(draft.delta)>limit||draft.speed>speed)return done('rejected','设备未开放所选POS流程或参数超出范围；未申请控制权');
        }
        if(explicitMode&&fields.switch!=='1')return done('rejected','设备未开放受保护模式切换');
      }
      if(stepIndex===0&&desiredMode&&fields.mode!==desiredMode&&!explicitMode){
        if(fields.switch!=='1'||!['1','2'].includes(fields.mode))return done('rejected','设备未开放模式切换，或当前模式未知');
        steps.unshift('acquire',desiredMode==='1'?'mode_mit':'mode_pos','release');
        operation=steps[0];modePreparation=true;
      }
      const unowned=['preflight','configure','acquire'].includes(operation);
      if(fields.owner!==(unowned?'0':'2'))return done('rejected','控制权与本流程不匹配，请先结束原会话');
      if(operation==='preflight'){
        if(fields.preflight!=='1'||fields.mode!=='1')return done('rejected','设备未开放MIT静载预检；当前模式不匹配');
      }else if(fields.fresh!=='1'&&!remoteDefersFeedback(operation,fields))return done('rejected','缺少新鲜标准反馈且设备未开放该操作的预检意图，停止后续步骤');
      if(desiredMode&&!(modePreparation&&stepIndex<2)&&fields.mode!==desiredMode)return done('rejected','模式回读与所选测试不匹配，后续步骤已停止');
      if(['mode_mit','mode_pos'].includes(operation)&&fields.switch!=='1')return done('rejected','设备未开放受保护模式切换');
      if(['configure','hold'].includes(operation)&&fields.static!=='1')return done('rejected','设备未开放静载能力');
      if(operation==='pos'||operation==='pos_load'){
        const loaded=operation==='pos_load',limit=Number(fields[loaded?'load_delta':'max_delta']),speed=Number(fields[loaded?'load_speed':'max_speed']);
        if(fields[loaded?'load':'pos']!=='1'||!Number.isFinite(limit)||limit<=0||!Number.isFinite(speed)||speed<=0||
          Math.abs(draft.delta)>limit||draft.speed>speed)return done('rejected','设备能力或运动参数范围不满足');
      }
      options.observe(operation,caps);
      const parameters={...draft,operation};
      if((draft.operation==='pos_load'||draft.operation==='home')&&operation!==draft.operation){parameters.kp=120;parameters.kd=.3;}
      if(!options.isCurrent())return done('cancelled','启动意图已取消');
      pendingOperation=operation;
      const submitted=await options.request({...parameters,token:Number(fields.token),motorId:7,coordinate:'output-rad'});
      pendingId=submitted.requestId;
      if(!options.isCurrent())return done('cancelled','提交后的连接已变化，仅可恢复原事务',pendingId);
      options.observe(operation,submitted);
      if(submitted.result!=='queued')return done(submitted.result==='rejected'?'rejected':'unknown','设备未确认入队；未重复提交',submitted.result==='rejected'?0:pendingId);
      let terminal:RemoteReceipt|null=null;
      const result=await monitorRemoteTransaction(pendingId,{
        request:options.request,wait:options.wait,isCurrent:options.isCurrent,
        observe:receipt=>{terminal=receipt;options.observe(operation,receipt);},
      });
      const completion=terminal as RemoteReceipt|null;
      if(['completed','failed','rejected'].includes(result)&&(!completion||!remoteTerminalConfirmed(completion,operation))){
        // 已知失败和安全失能是独立事实；保留原事务编号即可维持跨页面运动锁。
        if(completion?.result==='failed'&&completion.fields.accepted==='1')
          return done('failed',describeRemoteTerminal(completion,operation),pendingId);
        return done('unknown',completion?describeRemoteTerminal(completion,operation):'终态缺少必要的准入或失能确认，停止后续步骤',pendingId);
      }
      if(result!=='completed')return done(result,completion?describeRemoteTerminal(completion,operation):`${operation}未成功，后续步骤已停止`,['failed','rejected'].includes(result)?0:pendingId);
      pendingId=0;
    }
    return done('completed','本次流程已收到板端完成回执');
  }catch(error){return done('unknown',String(error)+'；后续步骤已停止，动作未重发',pendingId);}
}
