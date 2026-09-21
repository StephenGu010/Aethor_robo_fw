function report = run_adrc_client_tests(outputDirectory)
%RUN_ADRC_CLIENT_TESTS Verify the USB client using memory-only transport and clock.
% This entry never constructs serialport or calls MATLAB device APIs.
if nargin < 1, outputDirectory = tempname; end
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
checks = 0;
client = AdrcClient();
assert(~client.Connected && ~client.RunActive); checks = checks + 1;
fake = makeFixture();
assert(isempty(fake.WrittenLines)); checks = checks + 1;
client.connect(fake.callbacks());
assert(client.LastIssuedId == 40 && numel(fake.WrittenLines) == 1); checks = checks + 1;
assert(strcmp(fake.WrittenLines{1}, 'adrc status')); checks = checks + 1;
expectError(@()client.run(), 'AdrcClient:NotReady'); checks = checks + 1;
assert(~any(contains(fake.WrittenLines, ' adrc run'))); checks = checks + 1;
expectError(@()client.configure('evidence', struct('verified', 1)), 'AdrcClient:BadConfiguration'); checks = checks + 1;
client.configure('identify', struct('torque', -0.01, 'pulse_ms', 40));
assert(sum(startsWith(fake.WrittenLines, '41 adrc config group=identify ')) == 1); checks = checks + 1;
expectError(@()client.prepare(1), 'AdrcClient:ExecutionFailed'); checks = checks + 1;
assert(client.LastTerminal.Fields.result == 3); checks = checks + 1;
client.disconnect();

% A successful finite run keeps the 100 ms lease using separate increasing IDs.
fake = makeFixture(); fake.State.qualified = 1; fake.State.state = 1;
client = AdrcClient(); client.connect(fake.callbacks());
terminal = client.run();
assert(terminal.Fields.result == 0 && terminal.Fields.disabled == 1); checks = checks + 1;
heartbeatRows = find(contains(fake.WrittenLines, ' adrc heartbeat'));
assert(numel(heartbeatRows) == 3); checks = checks + 1;
heartbeatTimes = fake.WriteTimes(heartbeatRows);
assert(all(abs(heartbeatTimes - [0.1 0.2 0.3]) < 1e-8)); checks = checks + 1;
assert(all(diff(mutationIds(fake.WrittenLines)) > 0)); checks = checks + 1;
assert(~client.RunActive); checks = checks + 1;

% requestStop can be invoked by a GUI callback while synchronous run pumps events.
fake = makeFixture(); fake.State.qualified = 1; fake.State.state = 1;
client = AdrcClient(); client.connect(fake.callbacks());
fake.OnRead = @(transport)requestEarlyStop(transport, client);
terminal = client.run();
assert(terminal.Fields.disabled == 1 && any(contains(fake.WrittenLines, ' adrc stop'))); checks = checks + 1;
assert(fake.Clock < 0.35); checks = checks + 1;

% STOP DONE only acknowledges supervisor handling; confirmation needs fresh status.
fake = makeFixture(); fake.State.state = 2; fake.State.disabled = 0; fake.State.frozen = 0;
fake.State.active_id = 12;
client = AdrcClient(); client.connect(fake.callbacks());
stopped = client.stop();
assert(stopped.status.disabled == 1 && fake.Clock >= 0.04); checks = checks + 1;

% Lost RUN acknowledgement and lost heartbeat completion cause bounded failure, never a RUN retry.
fake = makeFixture(); fake.State.qualified = 1; fake.State.state = 1; fake.State.drop_run_ack = true;
fake.State.drop_run_done = true;
client = AdrcClient(); client.connect(fake.callbacks());
expectError(@()client.run(), 'AdrcClient:AckTimeout'); checks = checks + 1;
assert(sum(contains(fake.WrittenLines, ' adrc run')) == 1); checks = checks + 1;
assert(any(contains(fake.WrittenLines, ' adrc stop')) && client.LastStopAttempt.sent); checks = checks + 1;
assert(~client.Connected && ~client.LastStopAttempt.confirmed); checks = checks + 1;
fake = makeFixture(); fake.State.qualified = 1; fake.State.state = 1;
fake.State.run_seconds = 0.8; fake.State.drop_heartbeat_done = true;
client = AdrcClient(); client.connect(fake.callbacks());
expectError(@()client.run(), 'AdrcClient:HeartbeatTimeout'); checks = checks + 1;
assert(fake.Clock <= 0.31 && any(contains(fake.WrittenLines, ' adrc stop'))); checks = checks + 1;

% A disconnect cannot be reported as a confirmed stop.
fake = makeFixture(); fake.State.qualified = 1; fake.State.state = 1; fake.DisconnectAt = 0.15;
client = AdrcClient(); client.connect(fake.callbacks());
expectError(@()client.run(), 'AdrcClient:Disconnected'); checks = checks + 1;
assert(~client.LastStopAttempt.confirmed && ~client.Connected); checks = checks + 1;

% Firmware rejection and consumed request IDs are surfaced without automatic actuation retry.
fake = makeFixture(); fake.State.reject_command = 'clear';
client = AdrcClient(); client.connect(fake.callbacks());
expectError(@()client.clearFault(), 'AdrcClient:FirmwareRejected'); checks = checks + 1;
assert(client.LastIssuedId == 41 && sum(contains(fake.WrittenLines, ' adrc clear')) == 1); checks = checks + 1;
fake = makeFixture(); fake.State.last_id = double(intmax('uint32'));
client = AdrcClient(); client.connect(fake.callbacks());
expectError(@()client.clearFault(), 'AdrcClient:IdExhausted'); checks = checks + 1;
assert(numel(fake.WrittenLines) == 1); checks = checks + 1;

% Frozen export preserves uint64 device timestamps beyond double's exact integer range.
fake = makeFixture(); client = AdrcClient(); client.connect(fake.callbacks());
csvPath = [tempname(outputDirectory) '.csv'];
trace = client.exportTrace(csvPath);
assert(height(trace) == 2 && isa(trace.t, 'uint64')); checks = checks + 1;
expectedTimestamp = bitshift(uint64(1), 53) + uint64(1);
assert(trace.t(1) == expectedTimestamp && trace.t(2) == expectedTimestamp + uint64(4000)); checks = checks + 1;
assert(isfile(csvPath) && isfile([csvPath '.metadata.json'])); checks = checks + 1;
fake.State.disabled = 0; fake.State.state = 2;
expectError(@()client.exportTrace(fullfile(outputDirectory, 'must_not_exist.csv')), 'AdrcClient:TraceUnavailable'); checks = checks + 1;
assert(~isfile(fullfile(outputDirectory, 'must_not_exist.csv'))); checks = checks + 1;

report = struct('passed', true, 'checks', checks, 'hardwareOpened', false, ...
    'scope', 'memory_transport_protocol_contract_only', 'outputDirectory', outputDirectory);
fprintf('ADRC_CLIENT_TESTS_PASSED checks=%d hardwareOpened=0\n', checks);
end

function fake = makeFixture()
%MAKEFIXTURE Build an explicitly synthetic firmware response model.
fake = AdrcMemoryTransport();
fake.State = struct('last_id', 40, 'state', 0, 'fault', 0, 'motor', 1, ...
    'qualified', 0, 'disabled', 1, 'frozen', 1, 'count', 2, 'overflow', 0, ...
    'active_id', 0, 'run_seconds', 0.35, 'done_at', Inf, 'disabled_at', Inf, ...
    'drop_run_ack', false, 'drop_run_done', false, 'drop_heartbeat_done', false, ...
    'reject_command', '', 'stop_requested', false);
fake.OnWrite = @firmwareReply;
end

function firmwareReply(fake, line)
%FIRMWAREREPLY Emit the real firmware's response grammar without emulating hardware qualification.
words = strsplit(line);
if strcmp(words{1}, 'adrc'), identifier = 0; action = words{2};
else, identifier = str2double(words{1}); assert(strcmp(words{2}, 'adrc')); action = words{3}; end
if fake.Clock >= min(fake.State.done_at, fake.State.disabled_at)
    fake.State.state = 0; fake.State.disabled = 1; fake.State.frozen = 1; fake.State.active_id = 0;
end
if strcmp(action, 'status')
    fake.schedule(0, sprintf(['ok 0 adrc status state=%d fault=%d motor=%d qualified=%d disabled=%d ' ...
        'frozen=%d count=%d overflow=%d active_id=%d last_id=%.0f'], fake.State.state, fake.State.fault, ...
        fake.State.motor, fake.State.qualified, fake.State.disabled, fake.State.frozen, fake.State.count, ...
        fake.State.overflow, fake.State.active_id, fake.State.last_id));
    return;
end
if strcmp(action, 'trace')
    index = str2double(extractAfter(line, 'index='));
    timestamp = bitshift(uint64(1), 53) + uint64(1) + uint64(index * 4000);
    fake.schedule(0, sprintf(['ok 0 adrc trace index=%d t=%s fb=%s ref=0.1 pos=0 vel=0.01 ' ...
        'raw=0.002 req=0.002 sent=0.0019 z1=0.01 z2=0 state=2 fault=0 flags=9 seq=%d motor=1'], ...
        index, char(string(timestamp)), char(string(timestamp)), index));
    return;
end
assert(identifier > fake.State.last_id && identifier <= double(intmax('uint32')));
if strcmp(action, fake.State.reject_command)
    fake.schedule(0, sprintf('error %.0f adrc code=not_ready', identifier)); return;
end
fake.State.last_id = identifier;
if ~(strcmp(action, 'run') && fake.State.drop_run_ack)
    fake.schedule(0, sprintf('ok %.0f adrc accepted=1', identifier));
end
if strcmp(action, 'run')
    fake.State.state = 2; fake.State.disabled = 0; fake.State.frozen = 0; fake.State.active_id = identifier;
    fake.State.done_at = fake.Clock + fake.State.run_seconds;
    if ~fake.State.drop_run_done
        fake.schedule(fake.State.run_seconds, sprintf('done %.0f adrc result=0 disabled=1 cancelled=0', identifier));
    end
elseif strcmp(action, 'stop')
    fake.State.disabled_at = fake.Clock + 0.04; fake.State.state = 3;
    if fake.State.active_id ~= 0
        fake.removePrefix(sprintf('done %.0f adrc', fake.State.active_id));
        fake.schedule(0.04, sprintf('done %.0f adrc result=0 disabled=1 cancelled=0', fake.State.active_id));
    end
    fake.schedule(0, sprintf('done %.0f adrc result=0 disabled=0 cancelled=0', identifier));
elseif strcmp(action, 'heartbeat')
    if ~fake.State.drop_heartbeat_done
        fake.schedule(0, sprintf('done %.0f adrc result=0 disabled=0 cancelled=0', identifier));
    end
elseif strcmp(action, 'prepare')
    result = 3 * ~fake.State.qualified;
    if fake.State.qualified, fake.State.state = 1; end
    fake.schedule(0, sprintf('done %.0f adrc result=%d disabled=1 cancelled=0', identifier, result));
else
    fake.schedule(0, sprintf('done %.0f adrc result=0 disabled=1 cancelled=0', identifier));
end
end

function requestEarlyStop(fake, client)
%REQUESTEARLYSTOP Model a GUI callback setting a stop flag while readLine yields.
if fake.Clock >= 0.15 && ~fake.State.stop_requested && client.RunActive
    fake.State.stop_requested = true; client.requestStop();
end
end

function identifiers = mutationIds(lines)
%MUTATIONIDS Collect explicitly numbered commands for monotonic-ID assertions.
identifiers = [];
for index = 1:numel(lines)
    match = regexp(lines{index}, '^(\d+) adrc ', 'tokens', 'once');
    if ~isempty(match), identifiers(end + 1) = str2double(match{1}); end %#ok<AGROW>
end
end

function expectError(callback, expectedIdentifier)
%EXPECTERROR Require a specific client failure instead of accepting unrelated exceptions.
try
    callback();
catch failure
    assert(strcmp(failure.identifier, expectedIdentifier), '%s: %s', failure.identifier, failure.message);
    return;
end
error('AdrcClientTest:ExpectedFailure', 'Expected %s.', expectedIdentifier);
end
