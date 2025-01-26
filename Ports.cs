using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;

namespace uZBVIUZV
{
    public partial class Ports : Form
    {
        public Ports()
        {
            CultureInfo culture = new CultureInfo("en-US");
            Thread.CurrentThread.CurrentCulture = culture;
            Thread.CurrentThread.CurrentUICulture = culture;
            InitializeComponent();
            comportCB.Text = "";
            comportCB.Items.Clear();
            String[] ports = SerialPort.GetPortNames();
            comportCB.Items.AddRange(ports);
            comportCB.SelectedIndex = 0;
            baudrateCB.SelectedIndex = 0;
/*            ConnectPort();
            comportCB.Enabled = false;
            baudrateCB.Enabled = false;
            scanBT.Enabled = false;
            connectBT.Text = "DISCONNECT";*/
        }

        private void scanBT_Click(object sender, EventArgs e)
        {
            comportCB.Text = "";
            comportCB.Items.Clear();
            String[] ports = SerialPort.GetPortNames();
            comportCB.Items.AddRange(ports);
        }

        private void connectBT_Click(object sender, EventArgs e)
        {
            if (connectBT.Text == "CONNECT")
            {
                if (ConnectPort() == true)
                {
                    comportCB.Enabled = false;
                    baudrateCB.Enabled = false;
                    scanBT.Enabled = false;
                }
                connectBT.Text = "DISCONNECT";
            }
            else
            {
                serialPort1.Close();
                connectBT.Enabled = true;
                comportCB.Enabled = true;
                baudrateCB.Enabled = true;
                scanBT.Enabled = true;
                connectBT.Text = "CONNECT";
            }
        }

        public bool ConnectPort()
        {
            try
            {
                if (comportCB.Text != "" || baudrateCB.Text != "")
                {
                    serialPort1.PortName = comportCB.Text;
                    serialPort1.BaudRate = Int32.Parse(baudrateCB.Text);
                    serialPort1.Open();
                    return true;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "Error!", MessageBoxButtons.OK);
            }
            return false;
        }

        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            this.Invoke(new EventHandler(serialPort1_DataReceived));
        }

        private void serialPort1_DataReceived(object sender, EventArgs e)
        {
            string dump = serialPort1.ReadLine();
            incomingTB.Text = incomingTB.Text + dump;
            incomingTB.SelectionStart = incomingTB.TextLength;
            incomingTB.ScrollToCaret();
        }

        private void clearBT_Click_1(object sender, EventArgs e)
        {
            incomingTB.Text = "";
        }

        private void KeyUpPress(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Enter)
            {
                try
                {
                    if (!(serialPort1.IsOpen)) { serialPort1.Open(); }
                    string message = outgoingTB.Text;
                    foreach (char c in message)
                    {
                        serialPort1.Write(c.ToString());
                        Thread.Sleep(20);
                    }
                    serialPort1.Write("\n");
                    outgoingTB.Text = "";
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message, "Error!", MessageBoxButtons.OK);
                }
            }
        }

        private void ScrolP_Scroll(object sender, ScrollEventArgs e)
        {
            ValP.Text = (ScrolP.Value / 10.0).ToString("F2");
        }

        private void ScrolO_Scroll(object sender, ScrollEventArgs e)
        {
            ValO.Text = (ScrolO.Value / 10.0).ToString("F2");
        }

        private void SendAng_Click(object sender, EventArgs e)
        {
            try
            {
                if (!(serialPort1.IsOpen)) { serialPort1.Open(); }
                string message = $"Ang:{ScrolO.Value / 10.0 * 0.01745329252:F4};{ScrolP.Value / 10.0 * 0.01745329252:F4}";
                foreach (char c in message)
                {
                    serialPort1.Write(c.ToString());
                    Thread.Sleep(20);
                }
                serialPort1.Write("\n");
                outgoingTB.Text = "";
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "Error!", MessageBoxButtons.OK);
            }
        }
    }
}
