classdef AdrcClient < handle
    %ADRCCLIENT Explicit single-motor ADRC commands over an injected line transport.
    % Construction performs no I/O. Only connect() queries status; configuration,
    % prepare(), run() and stop() require explicit calls. No method grants evidence.
    % run() is synchronous and pumps a 100 ms heartbeat plus requestStop() callbacks.
    properties (SetAccess = private)
        Connected = false
        RunActive = false
        LastIssuedId = 0
        LastStatus = struct()
        LastTerminal = struct()
        LastStopAttempt = struct('sent', false, 'confirmed', false, 'message', '')
    end
    properties (Access = private)
        Transport = struct()
        Busy = false
        Synchronized = false
        StopRequested = false
        LastClock = -Inf
    end
    properties (Constant, Access = private)
        AckTimeout = 0.2
        QueryTimeout = 0.2
        HeartbeatPeriod = 0.1
        HeartbeatTimeout = 0.2
        PollInterval = 0.01
    end
    methods
        function obj = AdrcClient()
            %ADRCCLIENT Create an inert client without a transport or serial connection.
        end

        function snapshot = connect(obj, transport)
            %CONNECT Attach explicit callbacks and synchronize using a read-only status request.
            obj.requireIdle();
            if obj.Connected, error('AdrcClient:AlreadyConnected', 'Disconnect before attaching another transport.'); end
            required = {'WriteLine','ReadLine','IsOpen','Close','Now','Sleep'};
            for index = 1:numel(required)
                if ~isstruct(transport) || ~isfield(transport, required{index}) || ~isa(transport.(required{index}), 'function_handle')
                    error('AdrcClient:BadTransport', 'Missing transport callback %s.', required{index});
                end
            end
            obj.Transport = transport; obj.LastClock = -Inf;
            obj.Connected = logical(transport.IsOpen()); obj.Synchronized = false;
            obj.requireLink();
            snapshot = obj.status();
        end

        function snapshot = connectSerial(obj, portName, baudRate)
            %CONNECTSERIAL Explicitly open the requested port; never choose a device automatically.
            obj.requireIdle();
            if obj.Connected, error('AdrcClient:AlreadyConnected', 'Disconnect first.'); end
            transport = AdrcSerialTransport(portName, baudRate);
            transport.open();
            try
                snapshot = obj.connect(transport.callbacks());
            catch failure
                transport.close(); rethrow(failure);
            end
        end

        function disconnect(obj)
            %DISCONNECT Close an idle connection; an active run must first receive requestStop().
            obj.requireIdle(); obj.invalidateLink();
        end

        function delete(obj)
            %DELETE Attempt STOP only for an unfinished explicit run, then close transport storage.
            if obj.RunActive, obj.bestEffortStop(); end
            obj.invalidateLink();
        end

        function snapshot = status(obj)
            %STATUS Read a fresh uncached snapshot and retain the maximum issued/admitted ID.
            obj.requireIdle(); obj.requireLink(); obj.Busy = true;
            cleanup = onCleanup(@()obj.releaseBusy()); %#ok<NASGU>
            try
                snapshot = obj.readStatus();
            catch failure
                obj.invalidateLink(); rethrow(failure);
            end
        end

        function snapshot = acquireMotor(obj, motor, timeoutSeconds)
            %ACQUIREMOTOR Explicitly reserve integrated motor 7 after measured firmware gates pass.
            if nargin < 3, timeoutSeconds = 1.0; end
            validateattributes(motor, {'numeric'}, {'real','finite','scalar','integer','>=',1,'<=',7});
            snapshot = obj.transitionOwner(sprintf('adrc acquire motor=%d', motor), ...
                'acquire', 'lcd', 'adrc', 'transferred', timeoutSeconds);
        end

        function snapshot = releaseMotor(obj, timeoutSeconds)
            %RELEASEMOTOR Explicitly request DISABLE and await newer feedback before LCD handback.
            if nargin < 2, timeoutSeconds = 1.0; end
            snapshot = obj.transitionOwner('adrc release', ...
                'release', 'adrc', 'lcd', 'released', timeoutSeconds);
        end

        function terminal = configure(obj, group, fields)
            %CONFIGURE Send an explicit whitelisted patch; firmware remains the final admission authority.
            body = obj.configurationBody(group, fields);
            terminal = obj.mutate(body, 1.0);
        end

        function terminal = prepare(obj, motor)
            %PREPARE Select one public motor number; missing measured qualification remains an error.
            validateattributes(motor, {'numeric'}, {'real','finite','scalar','integer','>=',1,'<=',7});
            terminal = obj.mutate(sprintf('adrc prepare motor=%d', motor), 1.0);
        end

        function terminal = clearFault(obj)
            %CLEARFAULT Explicitly request the firmware's fresh-disabled fault-clear gate.
            terminal = obj.mutate('adrc clear', 1.0);
        end

        function terminal = heartbeat(obj)
            %HEARTBEAT Explicitly refresh the lease without starting or enabling an experiment.
            terminal = obj.mutate('adrc heartbeat', 0.2);
        end

        function terminal = run(obj, timeoutSeconds)
            %RUN Explicitly run the already prepared motor while maintaining 100 ms heartbeats.
            % Unknown outcomes are never retried as another RUN. Cleanup attempts STOP,
            % records that it is unconfirmed, and disconnects after timeout/interruption.
            if nargin < 2, timeoutSeconds = 4.0; end
            obj.validateTimeout(timeoutSeconds);
            snapshot = obj.status();
            if snapshot.state ~= 1 || snapshot.qualified ~= 1 || snapshot.disabled ~= 1 || snapshot.active_id ~= 0 || ...
                    (isfield(snapshot, 'owner') && ~strcmp(snapshot.owner, 'adrc'))
                error('AdrcClient:NotReady', 'Firmware does not report qualified, prepared and physically disabled.');
            end
            obj.Busy = true; obj.RunActive = true; obj.StopRequested = false;
            obj.LastStopAttempt = struct('sent', false, 'confirmed', false, 'message', '');
            cleanup = onCleanup(@()obj.finishRun()); %#ok<NASGU>
            identifier = obj.sendMutation('adrc run');
            terminal = obj.waitTerminal(identifier, timeoutSeconds, true);
            obj.LastTerminal = terminal;
            if obj.integerField(terminal.Fields, 'disabled', 0, 1) == 1
                obj.RunActive = false;
            end
            obj.checkTerminal(terminal);
            if obj.RunActive
                error('AdrcClient:DisableUnconfirmed', 'RUN DONE did not confirm physical disable.');
            end
        end

        function requestStop(obj)
            %REQUESTSTOP Set a flag safe for GUI/timer callbacks while synchronous run() is pumping.
            if ~obj.RunActive, error('AdrcClient:NoActiveRun', 'Use stop() when this client is not inside run().'); end
            obj.StopRequested = true;
        end

        function result = stop(obj, timeoutSeconds)
            %STOP Await STOP handling and then fresh status proving actual selected-axis disable.
            if nargin < 2, timeoutSeconds = 1.0; end
            obj.validateTimeout(timeoutSeconds); obj.requireIdle(); obj.requireSynchronized();
            obj.Busy = true; cleanup = onCleanup(@()obj.releaseBusy()); %#ok<NASGU>
            deadline = obj.now() + timeoutSeconds; stopWritten = false;
            try
                identifier = obj.sendMutation('adrc stop');
                stopWritten = true;
                terminal = obj.waitTerminal(identifier, timeoutSeconds, false);
                obj.LastTerminal = terminal; obj.checkTerminal(terminal);
                while obj.now() < deadline
                    snapshot = obj.readStatus();
                    if snapshot.disabled == 1 && any(snapshot.state == [0 4]) && snapshot.active_id == 0
                        obj.LastStopAttempt = struct('sent', true, 'confirmed', true, 'message', 'fresh_disabled_status');
                        result = struct('terminal', terminal, 'status', snapshot); return;
                    end
                    obj.sleep(min(0.02, max(0, deadline - obj.now())));
                end
                error('AdrcClient:DisableUnconfirmed', 'STOP was handled but fresh disabled status was not received in time.');
            catch failure
                obj.LastStopAttempt = struct('sent', stopWritten, 'confirmed', false, 'message', failure.message);
                obj.invalidateLink(); rethrow(failure);
            end
        end

        function trace = exportTrace(obj, csvPath)
            %EXPORTTRACE Read only a frozen disabled trace and export CSV plus explicit source metadata.
            obj.requireIdle(); obj.requireSynchronized();
            if ~(ischar(csvPath) || (isstring(csvPath) && isscalar(csvPath)))
                error('AdrcClient:BadPath', 'CSV path must be one text value.');
            end
            csvPath = char(csvPath);
            if isfile(csvPath) || isfile([csvPath '.metadata.json'])
                error('AdrcClient:OutputExists', 'Trace output already exists; choose a new path.');
            end
            obj.Busy = true; cleanup = onCleanup(@()obj.releaseBusy()); %#ok<NASGU>
            try
                before = obj.readStatus(); obj.requireFrozen(before);
                template = struct('index',0,'t',uint64(0),'fb',uint64(0),'ref',0,'pos',0,'vel',0, ...
                    'raw',0,'req',0,'sent',0,'z1',0,'z2',0,'state',0,'fault',0,'flags',0,'seq',0,'motor',0);
                records = repmat(template, before.count, 1);
                for index = 0:before.count - 1
                    records(index + 1) = obj.readTraceRow(index, template);
                    if index > 0 && records(index + 1).t <= records(index).t
                        error('AdrcClient:TraceChanged', 'Trace timestamps are not strictly increasing.');
                    end
                end
                after = obj.readStatus(); obj.requireFrozen(after);
                if before.count ~= after.count || before.overflow ~= after.overflow || before.motor ~= after.motor
                    error('AdrcClient:TraceChanged', 'Trace status changed during export.');
                end
                if before.count > 0 && ~isequaln(obj.readTraceRow(0, template), records(1))
                    error('AdrcClient:TraceChanged', 'Trace origin changed during export.');
                end
                trace = struct2table(records);
            catch failure
                if ~strcmp(failure.identifier, 'AdrcClient:TraceUnavailable'), obj.invalidateLink(); end
                rethrow(failure);
            end
            source = 'injected_transport';
            if isfield(obj.Transport, 'Name'), source = obj.Transport.Name; end
            metadata = struct('source', source, 'before', before, 'after', after, ...
                'torqueMeaning', 'protocol_nominal_commands_not_measured_torque', ...
                'exclusiveClientRequired', true, 'recordCount', height(trace));
            writetable(trace, csvPath);
            file = fopen([csvPath '.metadata.json'], 'w');
            if file < 0, error('AdrcClient:ExportFailed', 'Could not create trace metadata.'); end
            fileCleanup = onCleanup(@()fclose(file)); %#ok<NASGU>
            fprintf(file, '%s\n', jsonencode(metadata));
        end
    end
    methods (Access = private)
        function snapshot = transitionOwner(obj, body, acknowledgementField, initialOwner, ...
                finalOwner, finalHandoff, timeoutSeconds)
            %TRANSITIONOWNER Send once, match the ACK, then poll measured owner status.
            obj.validateTimeout(timeoutSeconds); obj.requireIdle(); obj.requireSynchronized();
            obj.Busy = true; cleanup = onCleanup(@()obj.releaseBusy()); %#ok<NASGU>
            try
                snapshot = obj.readStatus();
                if ~isfield(snapshot, 'owner')
                    error('AdrcClient:OwnerUnsupported', 'Firmware does not expose integrated ownership.');
                end
                if ~strcmp(snapshot.owner, initialOwner)
                    error('AdrcClient:OwnerState', 'Firmware owner is %s, expected %s.', snapshot.owner, initialOwner);
                end
                deadline = obj.now() + timeoutSeconds;
                identifier = obj.sendMutation(body);
                ackDeadline = min(deadline, obj.now() + obj.AckTimeout);
                acknowledged = false;
                for iteration = 1:256
                    remaining = ackDeadline - obj.now();
                    if remaining <= 0, break; end
                    message = obj.readMessage(min(obj.PollInterval, remaining));
                    if isempty(message) || message.Id ~= identifier, continue; end
                    obj.requireAdrc(message);
                    if strcmp(message.Kind, 'error'), obj.rejectMessage(message); end
                    if ~strcmp(message.Kind, 'ok') || ~isfield(message.Fields, acknowledgementField) || ...
                            ~strcmp(message.Fields.(acknowledgementField), 'accepted')
                        error('AdrcClient:ProtocolError', 'Ownership request did not receive its matching ACK.');
                    end
                    acknowledged = true; break;
                end
                if ~acknowledged
                    error('AdrcClient:AckTimeout', 'No ownership ACK for request %.0f.', identifier);
                end
                for iteration = 1:3000
                    snapshot = obj.readStatus();
                    if strcmp(snapshot.owner, finalOwner) && strcmp(snapshot.handoff, finalHandoff)
                        return;
                    end
                    if strcmp(acknowledgementField, 'acquire') && strcmp(snapshot.handoff, 'unsafe')
                        error('AdrcClient:OwnerRejected', 'Firmware rejected the measured ADRC handoff.');
                    end
                    if strcmp(acknowledgementField, 'release') && strcmp(snapshot.handoff, 'timeout')
                        error('AdrcClient:OwnerReleaseUnconfirmed', 'LCD handback lacks new disabled feedback.');
                    end
                    remaining = deadline - obj.now();
                    if remaining <= 0, break; end
                    obj.sleep(min(0.02, remaining));
                end
                if strcmp(acknowledgementField, 'release')
                    error('AdrcClient:OwnerReleaseUnconfirmed', 'LCD handback was not confirmed before timeout.');
                end
                error('AdrcClient:OwnerTransitionTimeout', 'ADRC ownership was not confirmed before timeout.');
            catch failure
                recoverable = {'AdrcClient:OwnerUnsupported','AdrcClient:OwnerState', ...
                    'AdrcClient:OwnerRejected','AdrcClient:OwnerReleaseUnconfirmed', ...
                    'AdrcClient:OwnerTransitionTimeout','AdrcClient:FirmwareRejected', ...
                    'AdrcClient:IdExhausted'};
                if ~any(strcmp(failure.identifier, recoverable)), obj.invalidateLink(); end
                rethrow(failure);
            end
        end

        function terminal = mutate(obj, body, timeoutSeconds)
            %MUTATE Send one ID once and retain precise terminal errors without automatic retries.
            obj.requireIdle(); obj.requireSynchronized(); obj.Busy = true;
            cleanup = onCleanup(@()obj.releaseBusy()); %#ok<NASGU>
            try
                identifier = obj.sendMutation(body);
                terminal = obj.waitTerminal(identifier, timeoutSeconds, false);
                obj.LastTerminal = terminal; obj.checkTerminal(terminal);
            catch failure
                if ~any(strcmp(failure.identifier, {'AdrcClient:FirmwareRejected','AdrcClient:ExecutionFailed','AdrcClient:IdExhausted'}))
                    obj.invalidateLink();
                end
                rethrow(failure);
            end
        end

        function identifier = sendMutation(obj, body)
            %SENDMUTATION Advance the boot-local ID before writing so uncertain writes are never reused.
            if obj.LastIssuedId >= double(intmax('uint32'))
                error('AdrcClient:IdExhausted', 'The uint32 request ID space is exhausted; do not wrap or reuse IDs.');
            end
            obj.LastIssuedId = obj.LastIssuedId + 1; identifier = obj.LastIssuedId;
            obj.writeLine(sprintf('%.0f %s', identifier, body));
        end

        function terminal = waitTerminal(obj, identifier, timeoutSeconds, keepHeartbeat)
            %WAITTERMINAL Route interleaved RUN/heartbeat/STOP replies with independent bounded deadlines.
            started = obj.now(); deadline = started + timeoutSeconds;
            accepted = false; heartbeatId = 0; heartbeatDeadline = Inf;
            nextHeartbeat = started + obj.HeartbeatPeriod; stopId = 0;
            for iteration = 1:20000
                current = obj.now();
                if ~accepted && current >= started + obj.AckTimeout
                    error('AdrcClient:AckTimeout', 'No ACK or terminal for request %.0f.', identifier);
                end
                if current >= deadline, error('AdrcClient:DoneTimeout', 'No terminal for request %.0f.', identifier); end
                if heartbeatId ~= 0 && current >= heartbeatDeadline
                    error('AdrcClient:HeartbeatTimeout', 'Heartbeat completion was not received before its deadline.');
                end
                if keepHeartbeat && obj.StopRequested && stopId == 0
                    stopId = obj.sendMutation('adrc stop');
                    obj.LastStopAttempt = struct('sent', true, 'confirmed', false, 'message', 'awaiting_run_terminal');
                end
                if keepHeartbeat && current >= nextHeartbeat && heartbeatId == 0
                    heartbeatId = obj.sendMutation('adrc heartbeat');
                    heartbeatDeadline = current + obj.HeartbeatTimeout;
                    nextHeartbeat = current + obj.HeartbeatPeriod;
                end
                wait = min(obj.PollInterval, deadline - current);
                if ~accepted, wait = min(wait, started + obj.AckTimeout - current); end
                if keepHeartbeat && heartbeatId == 0, wait = min(wait, nextHeartbeat - current); end
                if heartbeatId ~= 0, wait = min(wait, heartbeatDeadline - current); end
                message = obj.readMessage(max(0, wait));
                if isempty(message), continue; end
                if message.Id == identifier
                    obj.requireAdrc(message);
                    if strcmp(message.Kind, 'error'), obj.rejectMessage(message); end
                    if strcmp(message.Kind, 'done')
                        terminal = message;
                        if stopId ~= 0 && obj.integerField(message.Fields, 'disabled', 0, 1) == 1
                            obj.LastStopAttempt.confirmed = true;
                        end
                        return;
                    end
                    if ~strcmp(message.Kind, 'ok') || obj.integerField(message.Fields, 'accepted', 1, 1) ~= 1
                        error('AdrcClient:ProtocolError', 'Expected an accepted ACK.');
                    end
                    accepted = true;
                elseif (heartbeatId ~= 0 && message.Id == heartbeatId) || (stopId ~= 0 && message.Id == stopId)
                    obj.requireAdrc(message);
                    if strcmp(message.Kind, 'error'), obj.rejectMessage(message); end
                    if strcmp(message.Kind, 'done')
                        obj.checkTerminal(message);
                        if message.Id == heartbeatId, heartbeatId = 0; heartbeatDeadline = Inf; end
                    elseif ~strcmp(message.Kind, 'ok') || obj.integerField(message.Fields, 'accepted', 1, 1) ~= 1
                        error('AdrcClient:ProtocolError', 'Invalid heartbeat/STOP acknowledgement.');
                    end
                elseif message.Id == 0 && strcmp(message.Kind, 'error')
                    obj.rejectMessage(message);
                end
            end
            error('AdrcClient:ProtocolError', 'Transport produced an unbounded reply burst without progress.');
        end

        function snapshot = readStatus(obj)
            %READSTATUS Validate every required firmware field before accepting ID synchronization.
            message = obj.query('adrc status', 'status');
            snapshot = message.Fields;
            names = {'state','fault','motor','qualified','disabled','frozen','count','overflow','active_id','last_id'};
            minima = [0 0 1 0 0 0 0 0 0 0];
            maxima = [4 11 7 1 1 1 1024 double(intmax('uint32')) double(intmax('uint32')) double(intmax('uint32'))];
            for index = 1:numel(names), obj.integerField(snapshot, names{index}, minima(index), maxima(index)); end
            if snapshot.active_id > snapshot.last_id, error('AdrcClient:ProtocolError', 'Active ID exceeds admitted ID.'); end
            if xor(isfield(snapshot, 'owner'), isfield(snapshot, 'handoff'))
                error('AdrcClient:ProtocolError', 'Incomplete integrated ownership status.');
            end
            if isfield(snapshot, 'owner') && ...
                    (~ischar(snapshot.owner) || ~ischar(snapshot.handoff) || ...
                     ~any(strcmp(snapshot.owner, {'lcd','acquiring','adrc','releasing'})) || ...
                     ~any(strcmp(snapshot.handoff, {'pending','transferred','released','unsafe','timeout'})))
                error('AdrcClient:ProtocolError', 'Invalid integrated ownership status.');
            end
            obj.LastIssuedId = max(obj.LastIssuedId, snapshot.last_id);
            obj.LastStatus = snapshot; obj.Synchronized = true;
        end

        function message = query(obj, body, expectedPath)
            %QUERY Use uncached ID-zero reads, with one outstanding query and no ambiguous retries.
            obj.writeLine(body); deadline = obj.now() + obj.QueryTimeout;
            for iteration = 1:256
                remaining = deadline - obj.now();
                if remaining <= 0, error('AdrcClient:QueryTimeout', 'Read-only query timed out.'); end
                message = obj.readMessage(min(obj.PollInterval, remaining));
                if isempty(message), continue; end
                if message.Id ~= 0, continue; end
                if strcmp(message.Kind, 'error'), obj.rejectMessage(message); end
                obj.requireAdrc(message);
                if ~strcmp(message.Kind, 'ok') || ~strcmp(message.Path, expectedPath)
                    error('AdrcClient:ProtocolError', 'Unexpected reply to %s.', body);
                end
                return;
            end
            error('AdrcClient:ProtocolError', 'Too many unrelated replies during query.');
        end

        function row = readTraceRow(obj, index, template)
            %READTRACEROW Preserve device uint64 timestamps and verify index/sequence identity.
            message = obj.query(sprintf('adrc trace index=%d', index), 'trace');
            names = fieldnames(template); row = template;
            for fieldIndex = 1:numel(names)
                name = names{fieldIndex};
                if ~isfield(message.Fields, name), error('AdrcClient:ProtocolError', 'Missing trace field %s.', name); end
                row.(name) = message.Fields.(name);
                if ~isnumeric(row.(name)) || ~isscalar(row.(name)), error('AdrcClient:ProtocolError', 'Invalid trace field %s.', name); end
            end
            if row.index ~= index || row.seq ~= index || ~isa(row.t, 'uint64') || ~isa(row.fb, 'uint64')
                error('AdrcClient:TraceChanged', 'Trace index, sequence or timestamp type is invalid.');
            end
        end

        function writeLine(obj, line)
            %WRITELINE Enforce firmware ASCII/160-byte request bounds before transport output.
            obj.requireLink();
            if numel(line) > 160 || any(double(line) < 32 | double(line) > 126)
                error('AdrcClient:BadRequest', 'Request exceeds the firmware ASCII line contract.');
            end
            try
                obj.Transport.WriteLine(line);
            catch failure
                obj.Connected = false;
                error('AdrcClient:Disconnected', 'Transport write failed: %s', failure.message);
            end
        end

        function message = readMessage(obj, timeoutSeconds)
            %READMESSAGE Bound each transport read and decode complete response lines only.
            obj.requireLink();
            try
                line = obj.Transport.ReadLine(timeoutSeconds);
            catch failure
                obj.Connected = false;
                error('AdrcClient:Disconnected', 'Transport read failed: %s', failure.message);
            end
            if isempty(line), message = []; else, message = obj.parseReply(line); end
        end

        function value = now(obj)
            %NOW Require a finite monotonic injected clock for every bounded operation.
            value = obj.Transport.Now();
            if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value < obj.LastClock
                error('AdrcClient:ClockError', 'Transport clock is not finite and monotonic.');
            end
            obj.LastClock = value;
        end

        function sleep(obj, seconds)
            %SLEEP Yield through the transport clock without a hidden hardware dependency.
            try, obj.Transport.Sleep(seconds);
            catch failure, error('AdrcClient:Disconnected', 'Transport wait failed: %s', failure.message); end
        end

        function requireLink(obj)
            %REQUIRELINK Refuse implicit connection or reconnect behavior.
            if ~obj.Connected || ~isfield(obj.Transport, 'IsOpen') || ~obj.Transport.IsOpen()
                error('AdrcClient:Disconnected', 'No open explicit transport connection.');
            end
        end

        function requireSynchronized(obj)
            %REQUIRESYNCHRONIZED Require an observed firmware high-water ID before mutations.
            obj.requireLink();
            if ~obj.Synchronized, error('AdrcClient:NotSynchronized', 'Connect and obtain status before mutation.'); end
        end

        function requireIdle(obj)
            %REQUIREIDLE Prevent reentrant operations from corrupting reply matching.
            if obj.Busy, error('AdrcClient:Busy', 'Another operation is active; use requestStop() during run().'); end
        end

        function releaseBusy(obj)
            %RELEASEBUSY Release the single-operation guard on return or exception.
            obj.Busy = false;
        end

        function finishRun(obj)
            %FINISHRUN Retain uncertainty after exceptions or Ctrl-C instead of claiming a safe stop.
            if obj.RunActive
                obj.bestEffortStop(); obj.invalidateLink();
            end
            obj.RunActive = false; obj.StopRequested = false; obj.Busy = false;
        end

        function bestEffortStop(obj)
            %BESTEFFORTSTOP Send at most one new STOP ID without waiting during cleanup.
            obj.LastStopAttempt = struct('sent', false, 'confirmed', false, 'message', 'transport_unavailable');
            try
                if isfield(obj.Transport, 'IsOpen') && obj.Transport.IsOpen() && obj.LastIssuedId < double(intmax('uint32'))
                    obj.LastIssuedId = obj.LastIssuedId + 1;
                    obj.Transport.WriteLine(sprintf('%.0f adrc stop', obj.LastIssuedId));
                    obj.LastStopAttempt.sent = true;
                    obj.LastStopAttempt.message = 'STOP_written_without_disable_confirmation';
                end
            catch failure
                obj.LastStopAttempt.message = failure.message;
            end
        end

        function invalidateLink(obj)
            %INVALIDATELINK Close ambiguous streams; a later explicit connect must resynchronize.
            obj.Connected = false; obj.Synchronized = false;
            if isfield(obj.Transport, 'Close')
                try, obj.Transport.Close(); catch, end
            end
        end
    end
    methods (Static, Access = private)
        function body = configurationBody(group, fields)
            %CONFIGURATIONBODY Whitelist exact firmware groups and forbid evidence or command injection.
            group = char(string(group));
            switch group
                case 'control', allowed = {'b0','wc','wo','target','mode','duration_ms'};
                case 'mapping', allowed = {'pos_scale','vel_scale','torque_scale'};
                case 'limits', allowed = {'torque_limit','torque_slew','reference_accel','near_zero'};
                case 'identify', allowed = {'torque','pulse_ms'};
                otherwise, error('AdrcClient:BadConfiguration', 'Unsupported configuration group.');
            end
            if ~isstruct(fields) || ~isscalar(fields) || isempty(fieldnames(fields))
                error('AdrcClient:BadConfiguration', 'Supply a nonempty scalar field structure.');
            end
            names = fieldnames(fields);
            if ~all(ismember(names, allowed)), error('AdrcClient:BadConfiguration', 'Unknown configuration key.'); end
            body = ['adrc config group=' group];
            for index = 1:numel(names)
                name = names{index}; value = fields.(name);
                if strcmp(name, 'mode')
                    value = char(string(value));
                    if ~any(strcmp(value, {'identify','pi','ladrc'})), error('AdrcClient:BadConfiguration', 'Invalid controller mode.'); end
                else
                    value = AdrcClient.decimalNumber(value);
                end
                body = [body ' ' name '=' value]; %#ok<AGROW>
            end
        end

        function text = decimalNumber(value)
            %DECIMALNUMBER Expand float32 round-trip notation because request numbers forbid exponents.
            if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ~isfinite(value) || ~isfinite(single(value))
                error('AdrcClient:BadConfiguration', 'Numeric values must be finite float32 scalars.');
            end
            text = sprintf('%.9g', double(single(value)));
            exponentAt = regexp(text, '[eE]', 'once');
            if isempty(exponentAt), return; end
            mantissa = text(1:exponentAt - 1); exponent = str2double(text(exponentAt + 1:end));
            signText = '';
            if mantissa(1) == '-', signText = '-'; mantissa = mantissa(2:end); end
            decimalAt = find(mantissa == '.', 1);
            if isempty(decimalAt), point = numel(mantissa); else, point = decimalAt - 1; end
            digits = mantissa(mantissa ~= '.'); point = point + exponent;
            if point <= 0, text = [signText '0.' repmat('0', 1, -point) digits];
            elseif point >= numel(digits), text = [signText digits repmat('0', 1, point - numel(digits))];
            else, text = [signText digits(1:point) '.' digits(point + 1:end)]; end
        end

        function message = parseReply(line)
            %PARSEREPLY Decode the actual ok/error/done grammar with bounded ASCII and unique fields.
            if ~(ischar(line) || (isstring(line) && isscalar(line))), error('AdrcClient:ProtocolError', 'Reply is not text.'); end
            line = char(line);
            if numel(line) > 520, error('AdrcClient:ProtocolError', 'Reply exceeds firmware capacity.'); end
            line = regexprep(line, '[\r\n]+$', '');
            if any(double(line) < 32 | double(line) > 126), error('AdrcClient:ProtocolError', 'Reply contains non-ASCII or control bytes.'); end
            words = strsplit(strtrim(line));
            if numel(words) < 3 || ~any(strcmp(words{1}, {'ok','error','done'})) || isempty(regexp(words{2}, '^\d+$', 'once'))
                error('AdrcClient:ProtocolError', 'Invalid response header.');
            end
            identifier = str2double(words{2});
            if identifier > double(intmax('uint32')), error('AdrcClient:ProtocolError', 'Response ID exceeds uint32.'); end
            fields = struct(); path = {};
            for index = 4:numel(words)
                token = regexp(words{index}, '^([a-z][a-z0-9_]*)=(.+)$', 'tokens', 'once');
                if isempty(token)
                    if ~isempty(fieldnames(fields)) || isempty(regexp(words{index}, '^[a-z][a-z0-9_]*$', 'once'))
                        error('AdrcClient:ProtocolError', 'Malformed response field.');
                    end
                    path{end + 1} = words{index}; %#ok<AGROW>
                    continue;
                end
                if isfield(fields, token{1}), error('AdrcClient:ProtocolError', 'Duplicate response field.'); end
                value = token{2};
                if any(strcmp(token{1}, {'t','fb'})), value = AdrcClient.uint64Decimal(value);
                elseif ~isempty(regexpi(value, '^[-+]?(?:(?:\d+\.?\d*|\.\d+)(?:e[-+]?\d+)?|nan|inf)$', 'once'))
                    value = str2double(value);
                end
                fields.(token{1}) = value;
            end
            message = struct('Kind', words{1}, 'Id', identifier, 'Namespace', words{3}, ...
                'Path', strjoin(path, ' '), 'Fields', fields, 'Raw', line);
        end

        function value = uint64Decimal(text)
            %UINT64DECIMAL Parse timestamps without any lossy conversion through double.
            if isempty(regexp(text, '^\d+$', 'once')), error('AdrcClient:ProtocolError', 'Invalid uint64 timestamp.'); end
            value = uint64(0); maximum = intmax('uint64');
            for index = 1:numel(text)
                digit = uint64(double(text(index)) - double('0'));
                if value > idivide(maximum - digit, uint64(10), 'floor')
                    error('AdrcClient:ProtocolError', 'Timestamp exceeds uint64.');
                end
                value = value * uint64(10) + digit;
            end
        end

        function value = integerField(fields, name, minimum, maximum)
            %INTEGERFIELD Reject missing, nonfinite or out-of-range integer protocol fields.
            if ~isfield(fields, name), error('AdrcClient:ProtocolError', 'Missing field %s.', name); end
            value = fields.(name);
            if ~isnumeric(value) || ~isscalar(value) || ~isfinite(value) || value < minimum || value > maximum || value ~= fix(value)
                error('AdrcClient:ProtocolError', 'Invalid integer field %s.', name);
            end
        end

        function requireAdrc(message)
            %REQUIREADRC Reject namespace confusion for matching request IDs.
            if ~strcmp(message.Namespace, 'adrc'), error('AdrcClient:ProtocolError', 'Response namespace is not adrc.'); end
        end

        function rejectMessage(message)
            %REJECTMESSAGE Preserve the firmware reason and consume no automatic retry budget.
            reason = 'missing_code';
            if isfield(message.Fields, 'code'), reason = char(string(message.Fields.code)); end
            error('AdrcClient:FirmwareRejected', 'Request %.0f rejected: %s.', message.Id, reason);
        end

        function checkTerminal(message)
            %CHECKTERMINAL Distinguish accepted execution from successful physical completion.
            result = AdrcClient.integerField(message.Fields, 'result', 0, 6);
            AdrcClient.integerField(message.Fields, 'disabled', 0, 1);
            cancelled = AdrcClient.integerField(message.Fields, 'cancelled', 0, 1);
            if result ~= 0 || cancelled ~= 0
                error('AdrcClient:ExecutionFailed', 'Request %.0f terminal result=%d cancelled=%d.', message.Id, result, cancelled);
            end
        end

        function requireFrozen(snapshot)
            %REQUIREFROZEN Authorize trace retrieval only from actual disabled frozen status.
            if snapshot.disabled ~= 1 || snapshot.frozen ~= 1 || snapshot.active_id ~= 0 || ~any(snapshot.state == [0 4])
                error('AdrcClient:TraceUnavailable', 'Trace is not frozen with confirmed disable.');
            end
        end

        function validateTimeout(value)
            %VALIDATETIMEOUT Keep user-supplied waits explicit, finite and bounded.
            validateattributes(value, {'numeric'}, {'real','finite','scalar','positive','<=',60});
        end
    end
end
