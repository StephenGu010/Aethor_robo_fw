classdef AdrcMemoryTransport < handle
    %ADRCMEMORYTRANSPORT Deterministic line transport and clock for offline tests.
    % This fixture has no device APIs; callbacks may model firmware or lost replies.
    properties
        Clock = 0
        Open = true
        WrittenLines = {}
        WriteTimes = []
        State = struct()
        OnWrite = []
        OnRead = []
        DisconnectAt = Inf
    end
    properties (Access = private)
        Events = struct('time', {}, 'line', {})
    end
    methods
        function obj = AdrcMemoryTransport()
            %ADRCMEMORYTRANSPORT Create an empty memory fixture without device access.
        end

        function transport = callbacks(obj)
            %CALLBACKS Expose the same injectable contract as the serial transport.
            transport = struct('WriteLine', @(line)obj.writeLine(line), ...
                'ReadLine', @(timeout)obj.readLine(timeout), 'IsOpen', @()obj.Open, ...
                'Close', @()obj.close(), 'Now', @()obj.Clock, ...
                'Sleep', @(duration)obj.sleep(duration), 'Name', 'memory_fixture');
        end

        function writeLine(obj, line)
            %WRITELINE Record one request and invoke the explicitly configured fake firmware.
            obj.requireOpen();
            obj.WrittenLines{end + 1} = char(line);
            obj.WriteTimes(end + 1) = obj.Clock;
            if ~isempty(obj.OnWrite), obj.OnWrite(obj, char(line)); end
        end

        function schedule(obj, delay, line)
            %SCHEDULE Queue a complete reply at a relative virtual time in stable order.
            assert(isscalar(delay) && isfinite(delay) && delay >= 0);
            obj.Events(end + 1) = struct('time', obj.Clock + delay, 'line', char(line));
            [~, order] = sort([obj.Events.time]); obj.Events = obj.Events(order);
        end

        function removePrefix(obj, prefix)
            %REMOVEPREFIX Cancel synthetic pending replies matching one explicit prefix.
            if ~isempty(obj.Events), obj.Events(startsWith({obj.Events.line}, prefix)) = []; end
        end

        function line = readLine(obj, timeout)
            %READLINE Advance only to the next reply or bounded deadline, pumping test callbacks.
            obj.requireOpen();
            assert(isscalar(timeout) && isfinite(timeout) && timeout >= 0);
            deadline = obj.Clock + timeout;
            if ~isempty(obj.Events), deadline = min(deadline, max(obj.Clock, obj.Events(1).time)); end
            obj.advance(deadline);
            line = '';
            if ~isempty(obj.Events) && obj.Events(1).time <= obj.Clock
                line = obj.Events(1).line; obj.Events(1) = [];
            end
        end

        function sleep(obj, duration)
            %SLEEP Advance the virtual clock without consuming queued replies.
            obj.requireOpen();
            assert(isscalar(duration) && isfinite(duration) && duration >= 0);
            obj.advance(obj.Clock + duration);
        end

        function close(obj)
            %CLOSE Mark this memory connection closed without issuing any command.
            obj.Open = false;
        end
    end
    methods (Access = private)
        function advance(obj, target)
            %ADVANCE Apply scheduled disconnects before yielding to an optional GUI-like callback.
            if obj.DisconnectAt <= target
                obj.Clock = max(obj.Clock, obj.DisconnectAt); obj.Open = false;
                error('AdrcMemoryTransport:Disconnected', 'Synthetic transport disconnected.');
            end
            obj.Clock = target;
            if ~isempty(obj.OnRead), obj.OnRead(obj); end
        end

        function requireOpen(obj)
            %REQUIREOPEN Fail reads and writes after a fixture connection is closed.
            if ~obj.Open, error('AdrcMemoryTransport:Disconnected', 'Memory transport is closed.'); end
        end
    end
end
