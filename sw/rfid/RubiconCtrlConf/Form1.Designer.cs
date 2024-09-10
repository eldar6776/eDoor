namespace RubiconCtrlConf
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.btnConnect = new System.Windows.Forms.Button();
            this.rtxTextBox1 = new System.Windows.Forms.RichTextBox();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.cbxCommPort = new System.Windows.Forms.ComboBox();
            this.cbxRubiconAddress = new System.Windows.Forms.ComboBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.btnClose = new System.Windows.Forms.Button();
            this.cbxCommBaudrate = new System.Windows.Forms.ComboBox();
            this.cbxInterfaceAddress = new System.Windows.Forms.ComboBox();
            this.cbxSelectCommand = new System.Windows.Forms.ComboBox();
            this.btnSendCommand = new System.Windows.Forms.Button();
            this.btnExit = new System.Windows.Forms.Button();
            this.btnClearText = new System.Windows.Forms.Button();
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // btnConnect
            // 
            this.btnConnect.Location = new System.Drawing.Point(195, 32);
            this.btnConnect.Name = "btnConnect";
            this.btnConnect.Size = new System.Drawing.Size(75, 23);
            this.btnConnect.TabIndex = 0;
            this.btnConnect.Text = "Open Comm";
            this.btnConnect.UseVisualStyleBackColor = true;
            this.btnConnect.Click += new System.EventHandler(this.btnConnect_Click);
            // 
            // rtxTextBox1
            // 
            this.rtxTextBox1.Location = new System.Drawing.Point(10, 198);
            this.rtxTextBox1.Name = "rtxTextBox1";
            this.rtxTextBox1.Size = new System.Drawing.Size(263, 128);
            this.rtxTextBox1.TabIndex = 1;
            this.rtxTextBox1.Text = "";
            this.rtxTextBox1.TextChanged += new System.EventHandler(this.rtxTextBox1_TextChanged);
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(10, 134);
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(179, 20);
            this.textBox1.TabIndex = 2;
            this.textBox1.Text = "card id";
            // 
            // progressBar1
            // 
            this.progressBar1.Location = new System.Drawing.Point(9, 361);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(263, 15);
            this.progressBar1.TabIndex = 3;
            // 
            // cbxCommPort
            // 
            this.cbxCommPort.FormattingEnabled = true;
            this.cbxCommPort.Location = new System.Drawing.Point(8, 34);
            this.cbxCommPort.Name = "cbxCommPort";
            this.cbxCommPort.Size = new System.Drawing.Size(87, 21);
            this.cbxCommPort.TabIndex = 4;
            this.cbxCommPort.Text = "select port";
            this.cbxCommPort.Enter += new System.EventHandler(this.cbxCommPort_GotFocus);
            // 
            // cbxRubiconAddress
            // 
            this.cbxRubiconAddress.FormattingEnabled = true;
            this.cbxRubiconAddress.Location = new System.Drawing.Point(8, 86);
            this.cbxRubiconAddress.Name = "cbxRubiconAddress";
            this.cbxRubiconAddress.Size = new System.Drawing.Size(86, 21);
            this.cbxRubiconAddress.TabIndex = 6;
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.label3);
            this.groupBox1.Controls.Add(this.label2);
            this.groupBox1.Controls.Add(this.label1);
            this.groupBox1.Controls.Add(this.btnClose);
            this.groupBox1.Controls.Add(this.cbxCommBaudrate);
            this.groupBox1.Controls.Add(this.cbxInterfaceAddress);
            this.groupBox1.Controls.Add(this.cbxRubiconAddress);
            this.groupBox1.Controls.Add(this.cbxCommPort);
            this.groupBox1.Controls.Add(this.btnConnect);
            this.groupBox1.Location = new System.Drawing.Point(2, 12);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(279, 113);
            this.groupBox1.TabIndex = 7;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "connection";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(99, 70);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(66, 13);
            this.label4.TabIndex = 13;
            this.label4.Text = "App address";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(10, 70);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(82, 13);
            this.label3.TabIndex = 12;
            this.label3.Text = "Reader address";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(99, 18);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(50, 13);
            this.label2.TabIndex = 11;
            this.label2.Text = "Baudrate";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(10, 18);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(49, 13);
            this.label1.TabIndex = 10;
            this.label1.Text = "Com port";
            // 
            // btnClose
            // 
            this.btnClose.Location = new System.Drawing.Point(195, 84);
            this.btnClose.Name = "btnClose";
            this.btnClose.Size = new System.Drawing.Size(75, 23);
            this.btnClose.TabIndex = 9;
            this.btnClose.Text = "Close Comm";
            this.btnClose.UseVisualStyleBackColor = true;
            this.btnClose.Click += new System.EventHandler(this.btnClose_Click);
            // 
            // cbxCommBaudrate
            // 
            this.cbxCommBaudrate.FormattingEnabled = true;
            this.cbxCommBaudrate.Items.AddRange(new object[] {
            "9600",
            "19200",
            "38400",
            "57600",
            "115200",
            "230400",
            "460800",
            "921600"});
            this.cbxCommBaudrate.Location = new System.Drawing.Point(99, 34);
            this.cbxCommBaudrate.Name = "cbxCommBaudrate";
            this.cbxCommBaudrate.Size = new System.Drawing.Size(87, 21);
            this.cbxCommBaudrate.TabIndex = 8;
            // 
            // cbxInterfaceAddress
            // 
            this.cbxInterfaceAddress.FormattingEnabled = true;
            this.cbxInterfaceAddress.Location = new System.Drawing.Point(99, 86);
            this.cbxInterfaceAddress.Name = "cbxInterfaceAddress";
            this.cbxInterfaceAddress.Size = new System.Drawing.Size(87, 21);
            this.cbxInterfaceAddress.TabIndex = 7;
            // 
            // cbxSelectCommand
            // 
            this.cbxSelectCommand.FormattingEnabled = true;
            this.cbxSelectCommand.Items.AddRange(new object[] {
            "GET_SYS_FLAG",
            "GET_CARD_CNT",
            "GET_CARD_PRESENT",
            "GET_EVENT_CNT",
            "GET_EVENT_LAST",
            "SET_SYS_TIME",
            "SET_SYS_RESTART",
            "SET_CARD_ONE",
            "SET_DOOR_OPEN",
            "SET_DOOR_TIME",
            "SET_DOOR_ENABLE",
            "SET_DOOR_DISABLE",
            "SET_BUZZER_ENABLE",
            "SET_BUZZER_DISABLE",
            "DELETE_CARD_ONE",
            "DELETE_CARD_ALL",
            "DELETE_EVENT_LAST",
            "DELETE_EVENT_ALL",
            "RESTART_ONE",
            "RESTART_ALL"});
            this.cbxSelectCommand.Location = new System.Drawing.Point(9, 171);
            this.cbxSelectCommand.Name = "cbxSelectCommand";
            this.cbxSelectCommand.Size = new System.Drawing.Size(179, 21);
            this.cbxSelectCommand.TabIndex = 8;
            this.cbxSelectCommand.Text = "select command";
            this.cbxSelectCommand.SelectedIndexChanged += new System.EventHandler(this.cbxSelectCommand_SelectedIndexChanged);
            // 
            // btnSendCommand
            // 
            this.btnSendCommand.Location = new System.Drawing.Point(197, 169);
            this.btnSendCommand.Name = "btnSendCommand";
            this.btnSendCommand.Size = new System.Drawing.Size(75, 23);
            this.btnSendCommand.TabIndex = 9;
            this.btnSendCommand.Text = "Send";
            this.btnSendCommand.UseVisualStyleBackColor = true;
            this.btnSendCommand.Click += new System.EventHandler(this.btnSendCommand_Click);
            // 
            // btnExit
            // 
            this.btnExit.Location = new System.Drawing.Point(197, 332);
            this.btnExit.Name = "btnExit";
            this.btnExit.Size = new System.Drawing.Size(75, 23);
            this.btnExit.TabIndex = 10;
            this.btnExit.Text = "Exit";
            this.btnExit.UseVisualStyleBackColor = true;
            this.btnExit.Click += new System.EventHandler(this.btnExit_Click);
            // 
            // btnClearText
            // 
            this.btnClearText.Location = new System.Drawing.Point(9, 332);
            this.btnClearText.Name = "btnClearText";
            this.btnClearText.Size = new System.Drawing.Size(75, 23);
            this.btnClearText.TabIndex = 11;
            this.btnClearText.Text = "Clear";
            this.btnClearText.UseVisualStyleBackColor = true;
            this.btnClearText.Click += new System.EventHandler(this.btnClearText_Click);
            // 
            // serialPort1
            // 
            this.serialPort1.BaudRate = 115200;
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(284, 388);
            this.Controls.Add(this.btnClearText);
            this.Controls.Add(this.btnExit);
            this.Controls.Add(this.btnSendCommand);
            this.Controls.Add(this.cbxSelectCommand);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.rtxTextBox1);
            this.Name = "Form1";
            this.Text = "Form1";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button btnConnect;
        private System.Windows.Forms.RichTextBox rtxTextBox1;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.ComboBox cbxCommPort;
        private System.Windows.Forms.ComboBox cbxRubiconAddress;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.Button btnSendCommand;
        private System.Windows.Forms.Button btnClose;
        private System.Windows.Forms.ComboBox cbxCommBaudrate;
        private System.Windows.Forms.ComboBox cbxInterfaceAddress;
        private System.Windows.Forms.Button btnExit;
        private System.Windows.Forms.Button btnClearText;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.ComboBox cbxSelectCommand;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label1;
    }
}

