classdef AdrcSerialTransport < handle
    %ADRCSERIALTRANSPORT Explicitly opened ASCII line transport for one selected port.
    % Construction and callbacks() never enumerate or open hardware. open() is an
    % explicit user action; this class itself never sends ADRC or motor commands.
    properties (SetAccess = private)
        PortName
        BaudRate
    end
    properties (Access = private)
        Port = []
        Buffer = ''
        ClockOrigin
    end
    methods
        function obj = AdrcSerialTransport(portName, baudRate)
            %ADRCSERIALTRANSPORT Validate explicit connection settings without serial I/O.
            if nargin < 2, baudRate = 115200; end
            if ~(ischar(portName) || (isstring(portName) && isscalar(portName))) || isempty(char(portName))
                error('AdrcSerialTransport:BadPort', 'Supply one explicit nonempty port name.');
            end
            if ~isnumeric(baudRate) || ~isscalar(baudRate) || ~isfinite(baudRate) || baudRate <= 0 || fix(baudRate) ~= baudRate
                error('AdrcSerialTransport:BadBaudRate', 'Baud rate must be a positive integer.');
            end
            obj.PortName = char(portName); obj.BaudRate = baudRate; obj.ClockOrigin = tic;
        end

        function open(obj)
            %OPEN Open exactly the supplied port only when explicitly requested.
            if obj.isOpen(), error('AdrcSerialTransport:AlreadyOpen', 'Port is already open.'); end
            obj.Port = serialport(obj.PortName, obj.BaudRate, 'Timeout', 0.05);
            obj.Buffer = ''; flush(obj.Port, 'input');
        end

        function transport = callbacks(obj)
            %CALLBACKS Return callbacks without opening or otherwise touching a device.
            transport = struct('WriteLine', @(line)obj.writeLine(line), ...
                'ReadLine', @(timeout)obj.readLine(timeout), 'IsOpen', @()obj.isOpen(), ...
                'Close', @()obj.close(), 'Now', @()toc(obj.ClockOrigin), ...
                'Sleep', @(duration)pause(duration), 'Name', 'serial_live_device');
        end

        function result = isOpen(obj)
            %ISOPEN Report whether this adapter retains a valid serialport handle.
            result = ~isempty(obj.Port) && isvalid(obj.Port);
        end

        function writeLine(obj, line)
            %WRITELINE Write one validated ASCII request with a single LF terminator.
            obj.requireOpen(); line = char(line);
            if numel(line) > 160 || any(double(line) < 32 | double(line) > 126)
                error('AdrcSerialTransport:BadLine', 'Invalid bounded ASCII request.');
            end
            write(obj.Port, uint8([line char(10)]), 'uint8');
        end

        function line = readLine(obj, timeout)
            %READLINE Accumulate complete bounded replies while yielding to MATLAB callbacks.
            obj.requireOpen();
            if ~isscalar(timeout) || ~isfinite(timeout) || timeout < 0
                error('AdrcSerialTransport:BadTimeout', 'Timeout must be finite and nonnegative.');
            end
            deadline = toc(obj.ClockOrigin) + timeout; line = '';
            while true
                delimiter = find(obj.Buffer == char(10), 1);
                if ~isempty(delimiter)
                    if delimiter > 522, error('AdrcSerialTransport:LineTooLong', 'Reply exceeds protocol capacity.'); end
                    line = obj.Buffer(1:delimiter - 1); obj.Buffer(1:delimiter) = [];
                    if ~isempty(line) && line(end) == char(13), line(end) = []; end
                    return;
                end
                if numel(obj.Buffer) > 521, error('AdrcSerialTransport:LineTooLong', 'Unterminated reply exceeds protocol capacity.'); end
                obj.requireOpen(); available = obj.Port.NumBytesAvailable;
                if available > 0
                    if available + numel(obj.Buffer) > 8192
                        error('AdrcSerialTransport:ReceiveOverflow', 'Reply burst exceeds bounded receive storage.');
                    end
                    obj.Buffer = [obj.Buffer char(read(obj.Port, available, 'uint8'))]; %#ok<AGROW>
                    continue;
                end
                remaining = deadline - toc(obj.ClockOrigin);
                if remaining <= 0, return; end
                pause(min(0.002, remaining));
            end
        end

        function close(obj)
            %CLOSE Release the port and buffered bytes without sending motion or STOP commands.
            if ~isempty(obj.Port)
                try, delete(obj.Port); catch, end
            end
            obj.Port = []; obj.Buffer = '';
        end

        function delete(obj)
            %DELETE Release owned connection resources without opening hardware.
            obj.close();
        end
    end
    methods (Access = private)
        function requireOpen(obj)
            %REQUIREOPEN Reject all I/O before explicit open or after disconnection.
            if ~obj.isOpen(), error('AdrcSerialTransport:Disconnected', 'Serial connection is closed.'); end
        end
    end
end
