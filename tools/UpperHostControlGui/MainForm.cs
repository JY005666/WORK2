using System;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace UpperHostControlGui;

public sealed class MainForm : Form
{
    private readonly SerialPort _serialPort = new();
    private readonly StringBuilder _rxBuffer = new();

    private readonly ComboBox _portCombo = new();
    private readonly ComboBox _baudCombo = new();
    private readonly Button _refreshPortButton = new();
    private readonly Button _connectButton = new();
    private readonly TextBox _distanceBox = new();
    private readonly TextBox _yawBox = new();
    private readonly TextBox _armBox = new();
    private readonly TextBox _claw1Box = new();
    private readonly TextBox _claw2Box = new();
    private readonly TextBox _rawCommandBox = new();
    private readonly RichTextBox _logBox = new();
    private readonly Label _statusLabel = new();

    public MainForm()
    {
        Text = "Upper Host Control";
        StartPosition = FormStartPosition.CenterScreen;
        MinimumSize = new Size(980, 700);
        Font = new Font("Microsoft YaHei UI", 10F, FontStyle.Regular, GraphicsUnit.Point);

        _serialPort.Encoding = Encoding.ASCII;
        _serialPort.NewLine = "\r\n";
        _serialPort.DataReceived += SerialPort_DataReceived;

        BuildUi();
        RefreshPorts();
        UpdateConnectionStatus();
    }

    protected override void OnFormClosing(FormClosingEventArgs e)
    {
        if (_serialPort.IsOpen) {
            _serialPort.Close();
        }
        base.OnFormClosing(e);
    }

    private void BuildUi()
    {
        var root = new TableLayoutPanel {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 5,
            Padding = new Padding(12),
        };
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        root.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
        root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        Controls.Add(root);

        root.Controls.Add(BuildConnectionPanel());
        root.Controls.Add(BuildAxisPanel());
        root.Controls.Add(BuildQuickPanel());
        root.Controls.Add(BuildLogPanel());
        root.Controls.Add(BuildCommandPanel());
    }

    private Control BuildConnectionPanel()
    {
        var panel = new FlowLayoutPanel {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
            Padding = new Padding(0, 0, 0, 8),
        };

        _portCombo.Width = 120;
        _baudCombo.Width = 120;
        _baudCombo.Items.AddRange(new object[] { "115200", "9600", "57600", "230400", "460800" });
        _baudCombo.Text = "115200";

        _refreshPortButton.Text = "Refresh";
        _refreshPortButton.AutoSize = true;
        _refreshPortButton.Click += (_, _) => RefreshPorts();

        _connectButton.Text = "Connect";
        _connectButton.AutoSize = true;
        _connectButton.Click += (_, _) => ToggleConnection();

        _statusLabel.Text = "Disconnected";
        _statusLabel.AutoSize = true;
        _statusLabel.Padding = new Padding(12, 8, 0, 0);

        panel.Controls.Add(new Label { Text = "Port", AutoSize = true, Padding = new Padding(0, 8, 0, 0) });
        panel.Controls.Add(_portCombo);
        panel.Controls.Add(new Label { Text = "Baud", AutoSize = true, Padding = new Padding(8, 8, 0, 0) });
        panel.Controls.Add(_baudCombo);
        panel.Controls.Add(_refreshPortButton);
        panel.Controls.Add(_connectButton);
        panel.Controls.Add(_statusLabel);
        return panel;
    }

    private Control BuildAxisPanel()
    {
        var panel = new TableLayoutPanel {
            Dock = DockStyle.Fill,
            ColumnCount = 3,
            AutoSize = true,
            Padding = new Padding(0, 0, 0, 8),
        };
        panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33F));
        panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33F));
        panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.34F));

        panel.Controls.Add(BuildAxisGroup("Distance Axis 0 (mm)", _distanceBox, "SET 0 {0}", "ZERO 0", "800"), 0, 0);
        panel.Controls.Add(BuildAxisGroup("Yaw Axis 2 (deg)", _yawBox, "SET 2 {0}", "ZERO 2", "0"), 1, 0);
        panel.Controls.Add(BuildAxisGroup("Arm Axis 3 (deg)", _armBox, "SET 3 {0}", "ZERO 3", "0"), 2, 0);

        return panel;
    }

    private Control BuildAxisGroup(string title, TextBox valueBox, string setTemplate, string zeroCmd, string defaultValue)
    {
        var group = new GroupBox {
            Text = title,
            Dock = DockStyle.Fill,
            Padding = new Padding(12),
            Margin = new Padding(6),
            AutoSize = true,
        };

        var layout = new FlowLayoutPanel {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
        };

        valueBox.Width = 120;
        valueBox.Text = defaultValue;

        var sendButton = new Button {
            Text = "Send",
            AutoSize = true,
        };
        sendButton.Click += (_, _) => SendFormattedCommand(setTemplate, valueBox.Text);

        var zeroButton = new Button {
            Text = "Zero",
            AutoSize = true,
        };
        zeroButton.Click += (_, _) => SendCommand(zeroCmd);

        layout.Controls.Add(new Label { Text = "Target", AutoSize = true, Padding = new Padding(0, 8, 0, 0) });
        layout.Controls.Add(valueBox);
        layout.Controls.Add(sendButton);
        layout.Controls.Add(zeroButton);
        group.Controls.Add(layout);
        return group;
    }

    private Control BuildQuickPanel()
    {
        var panel = new TableLayoutPanel {
            Dock = DockStyle.Fill,
            ColumnCount = 2,
            AutoSize = true,
            Padding = new Padding(0, 0, 0, 8),
        };
        panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));
        panel.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50F));

        var clawGroup = new GroupBox {
            Text = "Claw Servo",
            Dock = DockStyle.Fill,
            Padding = new Padding(12),
            Margin = new Padding(6),
            AutoSize = true,
        };
        var clawLayout = new FlowLayoutPanel {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
        };
        _claw1Box.Width = 90;
        _claw2Box.Width = 90;
        _claw1Box.Text = "85";
        _claw2Box.Text = "120";

        var claw1Button = new Button { Text = "Set Claw1", AutoSize = true };
        claw1Button.Click += (_, _) => SendFormattedCommand("CLAW 1 {0}", _claw1Box.Text);
        var claw2Button = new Button { Text = "Set Claw2", AutoSize = true };
        claw2Button.Click += (_, _) => SendFormattedCommand("CLAW 2 {0}", _claw2Box.Text);
        var clawAllButton = new Button { Text = "Set Both", AutoSize = true };
        clawAllButton.Click += (_, _) => SendCommand($"CLAWALL {_claw1Box.Text} {_claw2Box.Text}");

        clawLayout.Controls.Add(new Label { Text = "Claw1", AutoSize = true, Padding = new Padding(0, 8, 0, 0) });
        clawLayout.Controls.Add(_claw1Box);
        clawLayout.Controls.Add(claw1Button);
        clawLayout.Controls.Add(new Label { Text = "Claw2", AutoSize = true, Padding = new Padding(8, 8, 0, 0) });
        clawLayout.Controls.Add(_claw2Box);
        clawLayout.Controls.Add(claw2Button);
        clawLayout.Controls.Add(clawAllButton);
        clawGroup.Controls.Add(clawLayout);

        var commonGroup = new GroupBox {
            Text = "Common Commands",
            Dock = DockStyle.Fill,
            Padding = new Padding(12),
            Margin = new Padding(6),
            AutoSize = true,
        };
        var commonLayout = new FlowLayoutPanel {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
        };

        commonLayout.Controls.Add(CreateCommandButton("ALL Send", () =>
            SendCommand($"ALL {_distanceBox.Text} {_yawBox.Text} {_armBox.Text}")));
        commonLayout.Controls.Add(CreateCommandButton("GET", () => SendCommand("GET")));
        commonLayout.Controls.Add(CreateCommandButton("STREAM ON", () => SendCommand("STREAM ON")));
        commonLayout.Controls.Add(CreateCommandButton("STREAM OFF", () => SendCommand("STREAM OFF")));
        commonLayout.Controls.Add(CreateCommandButton("STOP", () => SendCommand("STOP")));
        commonLayout.Controls.Add(CreateCommandButton("ZERO ALL", () => SendCommand("ZERO ALL")));
        commonLayout.Controls.Add(CreateCommandButton("HELP", () => SendCommand("HELP")));
        commonLayout.Controls.Add(CreateCommandButton("Clear Log", ClearLog));
        commonGroup.Controls.Add(commonLayout);

        panel.Controls.Add(clawGroup, 0, 0);
        panel.Controls.Add(commonGroup, 1, 0);
        return panel;
    }

    private Control BuildLogPanel()
    {
        var group = new GroupBox {
            Text = "Serial Log",
            Dock = DockStyle.Fill,
            Padding = new Padding(12),
            Margin = new Padding(6),
        };

        _logBox.Dock = DockStyle.Fill;
        _logBox.ReadOnly = true;
        _logBox.BackColor = Color.White;
        _logBox.Font = new Font("Consolas", 10F, FontStyle.Regular, GraphicsUnit.Point);
        group.Controls.Add(_logBox);
        return group;
    }

    private Control BuildCommandPanel()
    {
        var panel = new FlowLayoutPanel {
            Dock = DockStyle.Fill,
            AutoSize = true,
            WrapContents = true,
            Padding = new Padding(0, 8, 0, 0),
        };

        _rawCommandBox.Width = 500;
        _rawCommandBox.Text = "SET 2 120";
        _rawCommandBox.KeyDown += RawCommandBox_KeyDown;

        var sendButton = new Button {
            Text = "Send Raw",
            AutoSize = true,
        };
        sendButton.Click += (_, _) => SendRawCommand();

        panel.Controls.Add(new Label { Text = "Raw", AutoSize = true, Padding = new Padding(0, 8, 0, 0) });
        panel.Controls.Add(_rawCommandBox);
        panel.Controls.Add(sendButton);
        return panel;
    }

    private Button CreateCommandButton(string text, Action action)
    {
        var button = new Button {
            Text = text,
            AutoSize = true,
            Margin = new Padding(3),
        };
        button.Click += (_, _) => action();
        return button;
    }

    private void RawCommandBox_KeyDown(object sender, KeyEventArgs e)
    {
        if (e.KeyCode == Keys.Enter) {
            e.SuppressKeyPress = true;
            SendRawCommand();
        }
    }

    private void SendRawCommand()
    {
        SendCommand(_rawCommandBox.Text);
        _rawCommandBox.SelectAll();
    }

    private void SendFormattedCommand(string template, string valueText)
    {
        if (string.IsNullOrWhiteSpace(valueText)) {
            AppendLog("LOCAL", "Empty input");
            return;
        }
        SendCommand(string.Format(template, valueText.Trim()));
    }

    private void RefreshPorts()
    {
        var current = _portCombo.Text;
        var ports = SerialPort.GetPortNames().OrderBy(static p => p).ToArray();

        _portCombo.Items.Clear();
        _portCombo.Items.AddRange(ports);

        if (!string.IsNullOrWhiteSpace(current) && ports.Contains(current)) {
            _portCombo.Text = current;
        } else if (ports.Length > 0) {
            _portCombo.SelectedIndex = 0;
        } else {
            _portCombo.Text = string.Empty;
        }
    }

    private void ToggleConnection()
    {
        if (_serialPort.IsOpen) {
            _serialPort.Close();
            AppendLog("LOCAL", "Serial port disconnected");
            UpdateConnectionStatus();
            return;
        }

        if (string.IsNullOrWhiteSpace(_portCombo.Text)) {
            MessageBox.Show(this, "Please select a serial port.", "Port Required",
                MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }

        if (!int.TryParse(_baudCombo.Text, out var baudRate)) {
            MessageBox.Show(this, "Invalid baud rate.", "Baud Error",
                MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }

        try {
            _serialPort.PortName = _portCombo.Text.Trim();
            _serialPort.BaudRate = baudRate;
            _serialPort.DataBits = 8;
            _serialPort.StopBits = StopBits.One;
            _serialPort.Parity = Parity.None;
            _serialPort.Handshake = Handshake.None;
            _serialPort.Open();
            AppendLog("LOCAL", $"Connected to {_serialPort.PortName} @ {baudRate}");
        } catch (Exception ex) {
            MessageBox.Show(this, ex.Message, "Open Serial Port Failed",
                MessageBoxButtons.OK, MessageBoxIcon.Error);
        }

        UpdateConnectionStatus();
    }

    private void UpdateConnectionStatus()
    {
        var connected = _serialPort.IsOpen;
        _connectButton.Text = connected ? "Disconnect" : "Connect";
        _statusLabel.Text = connected
            ? $"Connected: {_serialPort.PortName} @ {_serialPort.BaudRate}"
            : "Disconnected";
    }

    private void SendCommand(string command)
    {
        if (string.IsNullOrWhiteSpace(command)) {
            return;
        }

        if (!_serialPort.IsOpen) {
            MessageBox.Show(this, "Serial port is not connected.", "Not Connected",
                MessageBoxButtons.OK, MessageBoxIcon.Warning);
            return;
        }

        try {
            var line = command.Trim();
            _serialPort.Write(line + "\r\n");
            AppendLog("TX", line);
        } catch (Exception ex) {
            AppendLog("ERR", ex.Message);
        }
    }

    private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
    {
        try {
            var text = _serialPort.ReadExisting();
            if (string.IsNullOrEmpty(text)) {
                return;
            }

            lock (_rxBuffer) {
                _rxBuffer.Append(text);
                ExtractLinesFromBuffer();
            }
        } catch (Exception ex) {
            BeginInvoke(new Action(() => AppendLog("ERR", ex.Message)));
        }
    }

    private void ExtractLinesFromBuffer()
    {
        while (true) {
            var bufferText = _rxBuffer.ToString();
            var newlineIndex = bufferText.IndexOfAny(new[] { '\r', '\n' });
            if (newlineIndex < 0) {
                break;
            }

            var line = bufferText.Substring(0, newlineIndex).Trim();
            var nextIndex = newlineIndex + 1;
            while (nextIndex < bufferText.Length &&
                   (bufferText[nextIndex] == '\r' || bufferText[nextIndex] == '\n')) {
                nextIndex++;
            }

            _rxBuffer.Clear();
            _rxBuffer.Append(bufferText.Substring(nextIndex));

            if (!string.IsNullOrWhiteSpace(line)) {
                BeginInvoke(new Action(() => AppendLog("RX", line)));
            }
        }
    }

    private void AppendLog(string tag, string text)
    {
        var line = $"[{DateTime.Now:HH:mm:ss}] {tag}: {text}{Environment.NewLine}";
        _logBox.AppendText(line);
        _logBox.SelectionStart = _logBox.TextLength;
        _logBox.ScrollToCaret();
    }

    private void ClearLog()
    {
        _logBox.Clear();
    }
}
