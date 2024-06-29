using System;
using System.IO;
using System.IO.Ports;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Windows.Forms;

namespace RubiconCtrlConf
{
    public partial class Form1 : Form
    {
        string strRtxBox1Text = null;
        string strCmdSelected = null;
        Int32 rs485_interface_address = 0;
        Int32 controller_interface_address = 0;
        UInt16 checksum = 0;
        byte[] buffer = new byte[1200];

        /**
         *  ako ne potrefi proces odprve, ovaj će delegat sam sebe zvat upomoć 
         *  dok ga ne potrefi od druge
         */
        delegate void SetTextCallback(string text);
        /**
         *  jedan međuproces za dozivanje između procesa
         */ 
        private Thread CallingThread = null;

        public Form1()
        {
            InitializeComponent();
            
        }
       
        private void cbxCommPort_GotFocus(object sender, EventArgs e)
        {
            string[] AvailableCOMPorts;

            // Clear combobox
            this.cbxCommPort.Items.Clear();
            this.cbxCommPort.Text = "";

            // Determine the COM ports currently available
            AvailableCOMPorts = System.IO.Ports.SerialPort.GetPortNames();

            for (int j = 0; j < AvailableCOMPorts.Length; j++)
            {
                // Add it to the combo box.
                this.cbxCommPort.Items.Add(AvailableCOMPorts[j]);
            }
        }

        private void btnConnect_Click(object sender, EventArgs e)
        {
            if (this.cbxCommPort.Text.CompareTo(System.String.Empty) != 0)
            {
                serialPort1.PortName = this.cbxCommPort.Text;
                serialPort1.BaudRate = Convert.ToInt32(cbxCommBaudrate.Text);
                serialPort1.DataBits = 8;
                serialPort1.StopBits = System.IO.Ports.StopBits.One;
                serialPort1.Parity = System.IO.Ports.Parity.None;
                serialPort1.Handshake = System.IO.Ports.Handshake.None;
               
                serialPort1.Encoding = Encoding.GetEncoding(28591);
                System.Text.Encoding.GetEncoding(28591); 

                try
                {
                    serialPort1.Open();
                }
                catch (System.Exception)
                {
                    MessageBox.Show("Failed to open selected port.\nIs it open in another application?\n(" + ")");
                }
                if (serialPort1.IsOpen)
                {
                    serialPort1.RtsEnable = true;
                    serialPort1.DtrEnable = true;
                    serialPort1.WriteTimeout = 5000;
                    cbxCommBaudrate.Enabled = false;
                    cbxCommPort.Enabled = false;
                    cbxRubiconAddress.Enabled = false;
                    cbxInterfaceAddress.Enabled = false;
                    controller_interface_address = Convert.ToInt32(cbxRubiconAddress.Text);
                    rs485_interface_address = Convert.ToInt32(cbxInterfaceAddress.Text);
                }
            }            
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            serialPort1.DataReceived += new SerialDataReceivedEventHandler(serialPort1_DataReceived);
            this.cbxCommBaudrate.Text = "115200";
            this.cbxRubiconAddress.Text = "100";
            this.cbxInterfaceAddress.Text = "254";
        }

        private void btnClose_Click(object sender, EventArgs e)
        {
            try
            {
                serialPort1.Close();
            }
            catch (System.Exception)
            {
                MessageBox.Show("Failed to close port.");
            }

            btnConnect.Enabled = true;
            cbxCommPort.Enabled = true;
            cbxCommBaudrate.Enabled = true;
            cbxRubiconAddress.Enabled = true;
            cbxInterfaceAddress.Enabled = true;
            btnClose.Enabled = false;
        }

        private void btnExit_Click(object sender, EventArgs e)
        {
            if (serialPort1.IsOpen)
            {
                try
                {
                    serialPort1.Close();
                }
                catch (System.Exception)
                {
                    MessageBox.Show("Failed to close selected port.");
                }
            }
            serialPort1.DataReceived -= new
                             SerialDataReceivedEventHandler(serialPort1_DataReceived);

            Application.Exit();
        }

        private void btnClearText_Click(object sender, EventArgs e)
        {
            this.rtxTextBox1.Text = "";
            //strRtxBox1Text = "\f";
            //this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
            //this.CallingThread.Start();
        }

        private void cbxSelectCommand_SelectedIndexChanged(object sender, EventArgs e)
        {
            strCmdSelected = cbxSelectCommand.Text; 
        }

        /**
         *  po jedna ovakva funkcija za svaku kontrolu na formi
         *  da ne komplikujemo poziv multifunkcije
         */ 
        private void threadSetRtxBox1_Text()
        {
            this.SetRtxBox1_Text(strRtxBox1Text);
        }
        /**
         *  po jedna ovakva funkcija koja izvršava poziv
         *  od gornje funkcije kada naleti proces koji je
         *  i stvorio kontrolu, da ne bi bilo otimanja
         */
        private void SetRtxBox1_Text(string text)
        {
            /**
             * provjerava se id procesa koji je stvorio controlu ,a onda i id procesa koji je pozvao ovu funkciju
             * pa ako se razlikuju onda će funkcija pozvati samu sebe da bi promjenila id procesa koji je pozvao
             * ovu funkciju i tako ukrug dok god ne potrfi id procesa koji je stvorio kontrolu, te će onda 
             * napokon napisati slova u polje za text
             */ 
            if (this.rtxTextBox1.InvokeRequired)
            {
                SetTextCallback d = new SetTextCallback(SetRtxBox1_Text);
                this.Invoke(d, new object[] { text });
            }
            else
            {
                this.rtxTextBox1.Text += text;
            }
        }
    //BAJT 0 = SOH(standardni ASCII "start of header" kontrolni char, 1 decimalno, 0x01 hexadecimalno)
    //BAJT 1 = ADRESA RECEIVERA(adresa ČITAČA ili u odgovoru 0xFE adresa PC aplikacije) ili 0xFF brodkast adresa
    //BAJT 2 = ADRESA TRANSMITERA(0xFE adresa PC aplikacije ili u odgovoru adresa ČITAČA)
    //BAJT 3 = DUŽINA PAKETA(broj bajta korisnog dijela ovog paketa koji saljemo prijemniku pocevsi od i uključujući slijedeceg)
    //BAJT 4 = KOMANDA
    //                    GET_SYS_FLAG        0xA0 // traži flagove sistema
    //                    GET_CARD_CNT        0xA1 // broj kartica u eepromu
    //                    GET_CARD_PRESENT    0xA2 // traži kartice u eepromu
    //                    GET_EVENT_CNT       0xA3 // broj događaja u eepromu
    //                    GET_EVENT_LAST      0xA4 // traži zadnji događaj iz eeproma

    //                    SET_SYS_TIME        0xB0 // podesi vrijeme u BCD formatu: dan/datum/mjesec/godina/sat/minuta/sekunda
    //                    SET_SYS_RESTART     0xB1 // softwerski restart applikacije
    //                    SET_CARD_ONE        0xB2 // dodaj novu karticu u eeprom
    //                    SET_DOOR_OPEN       0xB3 // aktiviraj bravu
    //                    SET_DOOR_TIME       0xB4 // podesi vrijeme brave
    //                    SET_DOOR_ENABLE     0xB5 // omogući kontrolu brave
    //                    SET_DOOR_DISABLE    0xB6 // onemogući kontrolu brave
    //                    SET_BUZZER_ENABLE   0xB7 // omogući buzzer
    //                    SET_BUZZER_DISABLE  0xB8 // onemogući buzzer

    //                    DELETE_CARD_ONE     0xC0 // obrisi karticu iz eeproma
    //                    DELETE_CARD_ALL     0xC1 // obrisi sve kartice iz eeproma
    //                    DELETE_EVENT_LAST   0xC2 // obrisi događaj prema datom indexu iz eeproma
    //                    DELETE_EVENT_ALL    0xC3 // obrisi sve događaje iz eeproma
                        
    //BAJT 5,6... = BROJ KARTICE ILI VRIJEME(ako se šalje broj kartice ili vrijeme)

    //BAJT ZADNJI - 2 = MSB CEKSUMA KOMANDE(16 bitni zbir svih bajta paketa, do prije ovog bajta)
    //BAJT ZADNJI - 1 = LSB CEKSUMA KOMANDE
    //BAJT ZADNJI		= EOT(standardni ASCII "end of transmission" kontrolni char, 4 decimalno, 0x04 hexadecimalno)
        private void btnSendCommand_Click(object sender, EventArgs e)
        {
            if (!serialPort1.IsOpen) return;

            buffer[0] = 0x01;   // packet start identifier
            buffer[1] = Convert.ToByte(controller_interface_address & 0xff);   // receiver rs485 address reader
            buffer[2] = Convert.ToByte(rs485_interface_address & 0xff);   // sender rs485 address PC
            buffer[3] = 0x01;   // packet data lenght

            if (strCmdSelected == "GET_SYS_FLAG") buffer[4] = 0xa0;
            else if (strCmdSelected == "GET_CARD_CNT") buffer[4] = 0xa1;
            else if (strCmdSelected == "GET_CARD_PRESENT")
            {
                UInt32 input = Convert.ToUInt32(textBox1.Text);

                buffer[3] = 0x05;   // data lenght
                buffer[4] = 0xa2;
                buffer[5] = Convert.ToByte((input >> 24) & 0xFF);
                buffer[6] = Convert.ToByte((input >> 16) & 0xFF);
                buffer[7] = Convert.ToByte((input >> 8) & 0xFF);
                buffer[8] = Convert.ToByte(input & 0xFF);
            }
            else if (strCmdSelected == "GET_EVENT_CNT") buffer[4] = 0xa3;
            else if (strCmdSelected == "GET_EVENT_LAST") buffer[4] = 0xa4;
            else if (strCmdSelected == "SET_SYS_RESTART") buffer[4] = 0xb1;
            else if (strCmdSelected == "SET_DOOR_OPEN") buffer[4] = 0xb3;
            else if (strCmdSelected == "SET_DOOR_TIME")
            {
                UInt32 input = Convert.ToUInt32(textBox1.Text);
                buffer[3] = 0x02;   // data lenght
                buffer[4] = 0xb4;
                buffer[5] = Convert.ToByte(input & 0xFF);
            }
            else if (strCmdSelected == "SET_DOOR_ENABLE") buffer[4] = 0xb5;
            else if (strCmdSelected == "SET_DOOR_DISABLE") buffer[4] = 0xb6;
            else if (strCmdSelected == "SET_BUZZER_ENABLE") buffer[4] = 0xb7;
            else if (strCmdSelected == "SET_BUZZER_DISABLE") buffer[4] = 0xb8;
            else if (strCmdSelected == "DELETE_CARD_ONE")
            {
                UInt32 input = Convert.ToUInt32(textBox1.Text);
                buffer[3] = 0x05;   // data lenght
                buffer[4] = 0xc0;
                buffer[5] = Convert.ToByte((input >> 24) & 0xFF);
                buffer[6] = Convert.ToByte((input >> 16) & 0xFF);
                buffer[7] = Convert.ToByte((input >> 8) & 0xFF);
                buffer[8] = Convert.ToByte(input & 0xFF);
            }
            else if (strCmdSelected == "DELETE_CARD_ALL") buffer[4] = 0xc1;
            else if (strCmdSelected == "DELETE_EVENT_LAST")
            {
                buffer[4] = 0xa4; // prvo šalji komadu citaj zadnji dogadjaj
                
            }
            else if (strCmdSelected == "DELETE_EVENT_ALL") buffer[4] = 0xc3;
            else if (strCmdSelected == "SET_SYS_TIME")
            {
                DateTime dt = DateTime.Now; // Or whatever
                string day = dt.ToString("dd");
                string month = dt.ToString("MM");
                string year = dt.ToString("yy");
                string hours = dt.ToString("HH");
                string minute = dt.ToString("mm");
                string seconds = dt.ToString("ss");

                buffer[3] = 0x0d;   // lenght
                buffer[4] = 0xb0;   // command set date & time
                buffer[5] = Convert.ToByte(day[0]);
                buffer[6] = Convert.ToByte(day[1]);
                buffer[7] = Convert.ToByte(month[0]);
                buffer[8] = Convert.ToByte(month[1]);
                buffer[9] = Convert.ToByte(year[0]);
                buffer[10] = Convert.ToByte(year[1]);
                buffer[11] = Convert.ToByte(hours[0]);
                buffer[12] = Convert.ToByte(hours[1]);
                buffer[13] = Convert.ToByte(minute[0]);
                buffer[14] = Convert.ToByte(minute[1]);
                buffer[15] = Convert.ToByte(seconds[0]);
                buffer[16] = Convert.ToByte(seconds[1]);
            }
            else if (strCmdSelected == "SET_CARD_ONE")
            {
                UInt32 input = Convert.ToUInt32(textBox1.Text);

                buffer[3] = 0x05;   // data lenght
                buffer[4] = 0xb2;
                buffer[5] = Convert.ToByte((input >> 24) & 0xFF);
                buffer[6] = Convert.ToByte((input >> 16) & 0xFF);
                buffer[7] = Convert.ToByte((input >> 8) & 0xFF);
                buffer[8] = Convert.ToByte(input & 0xFF);
            }
            /**
             *  check if command byte not empty then send 
             */ 
            if (buffer[3] != 0x00)
            {
                checksum = 0;
                for (uint i = 0; i < (buffer[3] + 4); i++)
                {
                    checksum += buffer[i];
                }

                buffer[buffer[3] + 4] = Convert.ToByte(checksum / 256);
                buffer[buffer[3] + 5] = Convert.ToByte(checksum & 0xff);
                buffer[buffer[3] + 6] = 0x04;
                try
                {
                    serialPort1.Write(buffer, 0, buffer[3] + 7);
                }
                catch (System.Exception)
                {
                    MessageBox.Show("Failed to write to port.");
                }
            }
        }
        
        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            // This event handler fires each time data is received by the serial port.
            // Read available data from the serial port and display it on the form.
            // This event does not run in the UI thread, so need to 
            // use delegate function
            for (int i = 0; i < 255; i++)
            {
                buffer[i] = 0;
            }
            int bytes = serialPort1.BytesToRead;
            Thread.Sleep(10);
            serialPort1.Read(buffer, 0, 512);
            if (buffer[0] == Convert.ToByte(rs485_interface_address & 0xff))
            {
                byte[] dest = new byte[buffer.Length + 1];
                System.Buffer.BlockCopy(buffer, 0, dest, 1, buffer.Length);
                dest[0] = 0x01;
                System.Buffer.BlockCopy(dest, 0, buffer, 0, buffer.Length);
            }

                if ((buffer[0] == 0x01) &&
                (buffer[1] == Convert.ToByte(rs485_interface_address & 0xff)) &&
                (buffer[buffer[3] + 6U] == 0x04))
            {
                checksum = 0;
                for (uint i = 0; i < (buffer[3] + 4); i++)
                {
                    checksum += buffer[i];
                }
                if (((buffer[buffer[3] + 4U]) == Convert.ToByte(checksum / 256)) && 
                    ((buffer[buffer[3] + 5U]) == Convert.ToByte(checksum & 0xff)))
                {
                    if (buffer[4] == 0xa0)
                    {
                        strRtxBox1Text = "odgovor: GET_SYS_FLAG\n";
                        if((buffer[5]  & 0x01) != 0) strRtxBox1Text += "log lista nije prazna\n";
                        else strRtxBox1Text += "log lista prazna\n";
                        if ((buffer[5] & 0x02) != 0) strRtxBox1Text += "log lista puna\n";
                        if ((buffer[5] & 0x04) != 0) strRtxBox1Text += "citac aktivan\n";
                        else strRtxBox1Text += "citac blokiran\n";
                        if ((buffer[5] & 0x08) != 0) strRtxBox1Text += "buzzer aktivan\n";
                        else strRtxBox1Text += "buzzer blokiran\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();

                    }
                    else if (buffer[4] == 0xa1)
                    {
                        int card_cnt = (buffer[5] << 8) | buffer[6];
                        strRtxBox1Text = "odgovor: GET_CARD_CNT\n";
                        strRtxBox1Text += "memorisanih kartica: ";
                        strRtxBox1Text += Convert.ToString(card_cnt);
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xa2)
                    {
                        strRtxBox1Text = "odgovor: GET_CARD_PRESENT\n";
                        if (buffer[5] == 0x06)  strRtxBox1Text += "kartica je u memoriji";
                        else if (buffer[5] == 0x15) strRtxBox1Text += "kartica nije u memoriji";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xa3)
                    {
                        int event_cnt = (buffer[5] << 8) | buffer[6];
                        strRtxBox1Text = "odgovor: GET_EVENT_CNT\n";
                        strRtxBox1Text += "memorisanih dogadjaja: ";
                        strRtxBox1Text += Convert.ToString(event_cnt);
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xa4)
                    {
                        // ako je komanda GET_EVENT_LAST bila poslana iz komande za brisanje
                        // nakon odgovora pošalji zahtjev za brisanje u roku 100ms
                        // inače bi odgovor uvijek bio "vrijeme za brisanje isteklo"
                        // za ovo vrijeme 100ms nema novog upisa u eeprom da ne bi bio obrisan 
                        // dogadjaj noviji od ovog pročitanog 
                        if (strCmdSelected == "DELETE_EVENT_LAST")
                        {
                            strRtxBox1Text = "odgovor: GET_EVENT_LAST\n";
                            strRtxBox1Text += "adresa citaca: ";
                            strRtxBox1Text += Convert.ToString(buffer[2]);
                            strRtxBox1Text += "\n------------------------\n";
                            this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                            this.CallingThread.Start();
                            buffer[0] = 0x01;   // packet start identifier
                            buffer[1] = Convert.ToByte(controller_interface_address & 0xff);   // receiver rs485 address lsb
                            buffer[2] = Convert.ToByte(rs485_interface_address & 0xff);   // sender rs485 address msb
                            buffer[3] = 0x01; // packet data lenght
                            buffer[4] = 0xc2;
                            checksum = 0;
                            for (uint i = 0; i < 5; i++)
                            {
                                checksum += buffer[i];
                            }

                            buffer[5] = Convert.ToByte(checksum / 256);
                            buffer[6] = Convert.ToByte(checksum & 0xff);
                            buffer[7] = 0x04;
                            try
                            {
                                serialPort1.Write(buffer, 0, 8);
                            }
                            catch (System.Exception)
                            {
                                MessageBox.Show("Failed to write to port.");
                            }
                        }
                        // odgovor na komandu GET_EVENT_LAST kada ima logova u memoriji
                        else if (buffer[3] == 0x11)
                        {
                            int id = (buffer[5] << 8) | buffer[6];
                            int card = (buffer[8] << 24) | (buffer[9] << 16) | (buffer[10] << 8) | buffer[11];
                            strRtxBox1Text = "odgovor: GET_EVENT_LAST\n";
                            strRtxBox1Text += "zadnji log index:";
                            strRtxBox1Text += Convert.ToString(id);
                            strRtxBox1Text += '\n';
                            if (buffer[7] == 0x01) strRtxBox1Text += "kartica validna\n";
                            else if (buffer[7] == 0x02) strRtxBox1Text += "kartica nije validna\n";
                            else if (buffer[7] == 0x03) strRtxBox1Text += "kartica validna, citac blokiran\n";
                            strRtxBox1Text += "broj kartice: ";
                            strRtxBox1Text += Convert.ToString(card);
                            strRtxBox1Text += '\n';
                            int dan, mjesec, godina = 2000;
                            int sat, minuta, sekunda;
                            dan = (10 * (buffer[12] >> 4));
                            dan += buffer[12] & 0xf;
                            mjesec = (10 * (buffer[13] >> 4));
                            mjesec += buffer[13] & 0xf;
                            godina += (10 * (buffer[14] >> 4));
                            godina += buffer[14] & 0xf;
                            sat = (10 * (buffer[15] >> 4));
                            sat += buffer[15] & 0xf;
                            minuta = (10 * (buffer[16] >> 4));
                            minuta += buffer[16] & 0xf;
                            sekunda = (10 * (buffer[17] >> 4));
                            sekunda += buffer[17] & 0xf;
                            strRtxBox1Text += "datum:";
                            strRtxBox1Text += Convert.ToString(dan);
                            strRtxBox1Text += ".";
                            strRtxBox1Text += Convert.ToString(mjesec);
                            strRtxBox1Text += "."; 
                            strRtxBox1Text += Convert.ToString(godina);
                            strRtxBox1Text += "\n";
                            strRtxBox1Text += "vrijeme:";
                            strRtxBox1Text += Convert.ToString(sat);
                            strRtxBox1Text += ":";
                            strRtxBox1Text += Convert.ToString(minuta);
                            strRtxBox1Text += ":";
                            strRtxBox1Text += Convert.ToString(sekunda);
                            strRtxBox1Text += "\n------------------------\n";
                            this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                            this.CallingThread.Start();
                        }
                        // odgovor na komandu GET_EVENT_LAST kada je log lista prazna
                        else if ((buffer[3] == 0x02) && (buffer[5] == 0x02))
                        {
                            strRtxBox1Text = "odgovor: GET_EVENT_LAST\n";
                            strRtxBox1Text += "log lista prazna";
                            strRtxBox1Text += "\n------------------------\n";
                            this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                            this.CallingThread.Start();
                        }
                        // ovdje dolaze real time dogadjaji sa istim GET_EVENT_LAST (0xa4) bajtom u komandi
                        else if (buffer[3] == 0x06)
                        {
                            int card = (buffer[6] << 24) | (buffer[7] << 16) | (buffer[8] << 8) | buffer[9];
                            strRtxBox1Text = "novi dogadjaj\n";
                            strRtxBox1Text += "adresa citaca: ";
                            strRtxBox1Text += Convert.ToString(buffer[2]);
                            strRtxBox1Text += '\n';
                            if (buffer[5] == 0x01) strRtxBox1Text += "kartica validna\n";
                            else if (buffer[5] == 0x02) strRtxBox1Text += "kartica nije validna\n";
                            else if (buffer[5] == 0x03) strRtxBox1Text += "kartica validna - citac blokiran\n";
                            strRtxBox1Text += "broj kartice: ";
                            strRtxBox1Text += Convert.ToString(card);
                            strRtxBox1Text += "\n------------------------\n";
                            this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                            this.CallingThread.Start();
                            buffer[0] = 0x01;   // packet start identifier
                            buffer[1] = Convert.ToByte(controller_interface_address & 0xff);   // receiver rs485 address lsb
                            buffer[2] = Convert.ToByte(rs485_interface_address & 0xff);   // sender rs485 address msb
                            buffer[3] = 0x01; // packet data lenght
                            buffer[4] = 0x06; // ACK
                            checksum = 0;
                            for (uint i = 0; i < 5; i++)
                            {
                                checksum += buffer[i];
                            }

                            buffer[5] = Convert.ToByte(checksum / 256);
                            buffer[6] = Convert.ToByte(checksum & 0xff);
                            buffer[7] = 0x04;
                            try
                            {
                                serialPort1.Write(buffer, 0, 8);
                            }
                            catch (System.Exception)
                            {
                                MessageBox.Show("Failed to write to port.");
                            }
                        }
                    }
                    else if (buffer[4] == 0xb0)
                    {
                        strRtxBox1Text = "odgovor: SET_SYS_TIME\n";
                        strRtxBox1Text += "vrijeme i datum podeseni";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xB1)
                    {
                        strRtxBox1Text = "odgovor: SET_SYS_RESTART\n";
                        strRtxBox1Text += "citac restartovan";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xB2)
                    {
                        strRtxBox1Text = "odgovor: SET_CARD_ONE\n";
                        if (buffer[5] == 0x06) strRtxBox1Text += "kartica memorisana";
                        else if (buffer[5] == 0x15) strRtxBox1Text += "kartica nije memorisana";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xb3)
                    {
                        strRtxBox1Text = "odgovor: SET_DOOR_OPEN\n";
                        strRtxBox1Text += "brava aktivirana";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xb4)
                    {
                        strRtxBox1Text = "odgovor: SET_DOOR_TIME\n";
                        strRtxBox1Text += "vrijeme brave podeseno";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xb5)
                    {
                        strRtxBox1Text = "odgovor: SET_DOOR_ENABLE\n";
                        strRtxBox1Text += "brava omogucena";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xb6)
                    {
                        strRtxBox1Text = "odgovor: SET_DOOR_DISABLE\n";
                        strRtxBox1Text += "brava onemogucena";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xb7)
                    {
                        strRtxBox1Text = "odgovor: SET_BUZZER_ENABLE\n";
                        strRtxBox1Text += "buzzer omogucen";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xb8)
                    {
                        strRtxBox1Text = "odgovor: SET_BUZZER_DISABLE\n";
                        strRtxBox1Text += "buzzer onemogucen";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xC0)
                    {
                        strRtxBox1Text = "odgovor: DELETE_CARD_ONE\n";
                        if (buffer[5] == 0x06) strRtxBox1Text += "kartica izbrisana";
                        else if (buffer[5] == 0x15) strRtxBox1Text += "kartica nije pronadjena";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xC1)
                    {
                        strRtxBox1Text = "odgovor: DELETE_CARD_ALL\n";
                        strRtxBox1Text += "memorija kartica izbrisana";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xC2)
                    {
                        strRtxBox1Text = "odgovor: DELETE_EVENT_LAST\n";
                        if (buffer[5] == 0x00) strRtxBox1Text += "zadnji dogadjaj izbrisan";
                        else if (buffer[5] == 0x02) strRtxBox1Text += "lista prazna";
                        else if (buffer[5] == 0x03) strRtxBox1Text += "vrijeme za brisanje isteklo";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    else if (buffer[4] == 0xC3)
                    {
                        strRtxBox1Text = "odgovor: DELETE_EVENT_ALL\n";
                        strRtxBox1Text += "memorija dogadjaja izbrisana";
                        strRtxBox1Text += "\n------------------------\n";
                        this.CallingThread = new Thread(new ThreadStart(this.threadSetRtxBox1_Text));
                        this.CallingThread.Start();
                    }
                    
                }
            }
        }

        private void rtxTextBox1_TextChanged(object sender, EventArgs e)
        {
            rtxTextBox1.SelectionStart = rtxTextBox1.Text.Length;
            rtxTextBox1.ScrollToCaret();
        }

        
    }
}
