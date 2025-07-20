import ttkbootstrap as ttk
from ttkbootstrap.constants import *
import serial
import serial.tools.list_ports

class CDCControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Voltage & Current Control")
        self.root.geometry("620x650")
        self.ser = None

        self.voltage = ttk.DoubleVar(value=5.00)
        self.current = ttk.IntVar(value=500)
        self.port_var = ttk.StringVar()
        
        # Variables to store received values
        self.received_vout = ttk.DoubleVar(value=0.0)
        self.received_iout = ttk.DoubleVar(value=0.0)
        self.received_vin = ttk.DoubleVar(value=0.0)
        
        # On/Off state variable
        self.output_on = ttk.BooleanVar(value=False)

        self.build_gui()
    
    

    def build_gui(self):
        frame = ttk.Frame(self.root, padding=30)
        frame.pack(fill=BOTH, expand=YES)

        # Title
        ttk.Label(frame, text="STM32 USB CDC Control", font=("Segoe UI", 20, "bold")).pack(pady=15)

        # Serial Port Selector
        port_frame = ttk.Frame(frame)
        port_frame.pack(pady=10)
        ttk.Label(port_frame, text="Select COM Port:", font=("Segoe UI", 12)).pack(side=LEFT, padx=5)
        self.port_menu = ttk.Combobox(port_frame, textvariable=self.port_var, width=25, values=self.get_serial_ports(), font=("Segoe UI", 11))
        self.port_menu.pack(side=LEFT, padx=5)
        ttk.Button(port_frame, text="Connect", bootstyle=SUCCESS, command=self.connect_serial).pack(side=LEFT, padx=5)

        # Voltage Section
        ttk.Label(frame, text="Voltage (V)", font=("Segoe UI", 14)).pack(pady=(25, 5))
        self.voltage_slider = ttk.Scale(frame, from_=0.0, to=12.0, variable=self.voltage, length=450, bootstyle="info", command=self.slider_voltage_changed)
        self.voltage_slider.pack()
        v_box = ttk.Frame(frame)
        v_box.pack()
        self.voltage_display = ttk.Label(v_box, text="5.00 V", font=("Segoe UI", 12))
        self.voltage_display.pack(side=LEFT, padx=10)
        self.voltage_entry = ttk.Entry(v_box, width=10, font=("Segoe UI", 11))
        self.voltage_entry.insert(0, "5.00")
        self.voltage_entry.pack(side=LEFT)
        ttk.Button(v_box, text="Set", command=self.set_voltage_from_entry, bootstyle="info-outline").pack(side=LEFT, padx=5)

        # Current Section
        ttk.Label(frame, text="Current (mA)", font=("Segoe UI", 14)).pack(pady=(30, 5))
        self.current_slider = ttk.Scale(frame, from_=0, to=2000, variable=self.current, length=450, bootstyle="warning", command=self.slider_current_changed)
        self.current_slider.pack()
        c_box = ttk.Frame(frame)
        c_box.pack()
        self.current_display = ttk.Label(c_box, text="500 mA", font=("Segoe UI", 12))
        self.current_display.pack(side=LEFT, padx=10)
        self.current_entry = ttk.Entry(c_box, width=10, font=("Segoe UI", 11))
        self.current_entry.insert(0, "500")
        self.current_entry.pack(side=LEFT)
        ttk.Button(c_box, text="Set", command=self.set_current_from_entry, bootstyle="warning-outline").pack(side=LEFT, padx=5)

        # Send Button
        ttk.Button(frame, text="Send to STM32", bootstyle=PRIMARY, command=self.send_values, width=30).pack(pady=30)

        # On/Off Button
        self.onoff_button = ttk.Button(frame, text="Turn ON", bootstyle="success-outline", command=self.toggle_output, width=30)
        self.onoff_button.pack(pady=10)

        # Feedback Button
        ttk.Button(frame, text="Feedback", bootstyle=SECONDARY, command=self.send_feedback, width=30).pack(pady=10)

        # Received Values Display Section
        ttk.Label(frame, text="Received Values from STM32", font=("Segoe UI", 14, "bold")).pack(pady=(30, 10))
        
        received_frame = ttk.Frame(frame)
        received_frame.pack(pady=10)
        
        # VOUT display
        vout_frame = ttk.Frame(received_frame)
        vout_frame.pack(pady=5)
        ttk.Label(vout_frame, text="Output Voltage:", font=("Segoe UI", 11)).pack(side=LEFT, padx=5)
        self.vout_display = ttk.Label(vout_frame, text="0.00 V", font=("Segoe UI", 11, "bold"), bootstyle="info")
        self.vout_display.pack(side=LEFT, padx=10)
        
        # IOUT display
        iout_frame = ttk.Frame(received_frame)
        iout_frame.pack(pady=5)
        ttk.Label(iout_frame, text="Output Current:", font=("Segoe UI", 11)).pack(side=LEFT, padx=5)
        self.iout_display = ttk.Label(iout_frame, text="0.00 mA", font=("Segoe UI", 11, "bold"), bootstyle="warning")
        self.iout_display.pack(side=LEFT, padx=10)
        
        # VIN display
        vin_frame = ttk.Frame(received_frame)
        vin_frame.pack(pady=5)
        ttk.Label(vin_frame, text="Input Voltage:", font=("Segoe UI", 11)).pack(side=LEFT, padx=5)
        self.vin_display = ttk.Label(vin_frame, text="0.00 V", font=("Segoe UI", 11, "bold"), bootstyle="success")
        self.vin_display.pack(side=LEFT, padx=10)

        # Live Update Loop
        self.root.after(100, self.update_display)

    def get_serial_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port_var.get(), 115200, timeout=1)
            self.port_menu.config(state="disabled")
            print("Connected to", self.port_var.get())
        except Exception as e:
            print("Connection failed:", e)

    def send_values(self):
        if self.ser and self.ser.is_open:
            v = self.voltage.get()
            i = self.current.get()
            message = f"V:{v:.2f}\n"
            self.ser.write(message.encode())
            message = f"I:{int(i)}\n"
            self.ser.write(message.encode())
            print("Sent:", message.strip())

    def send_feedback(self):
        """Send feedback command to STM32"""
        if self.ser and self.ser.is_open:
            message = "F:1\n"
            self.ser.write(message.encode())
            print("Sent feedback:", message.strip())
        else:
            print("Serial port not connected")

    def toggle_output(self):
        """Toggle output on/off and send command to STM32"""
        if self.ser and self.ser.is_open:
            if self.output_on.get():
                # Currently ON, turn OFF
                message = "O:0\n"
                self.ser.write(message.encode())
                self.output_on.set(False)
                self.onoff_button.config(text="Turn ON", bootstyle="success-outline")
                print("Sent: Output OFF")
            else:
                # Currently OFF, turn ON
                message = "O:1\n"
                self.ser.write(message.encode())
                self.output_on.set(True)
                self.onoff_button.config(text="Turn OFF", bootstyle="danger")
                print("Sent: Output ON")
        else:
            print("Serial port not connected")

    def read_from_serial(self):
        """Read data from serial port if available"""
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8').strip()
                    if data:
                        print("Received:", data)
                        self.process_received_data(data)
                        return data
            except Exception as e:
                print("Error reading from serial:", e)
        return None

    def process_received_data(self, data):
        """Process received data from STM32"""
        try:
            # Parse comma-separated data format: VOUT:0.01,IOUT:1.17,VIN:0.00
            if "," in data:
                # Split by comma and process each part
                parts = data.split(",")
                for part in parts:
                    part = part.strip()
                    if part.startswith("VOUT:"):
                        voltage_str = part.split(":")[1]
                        voltage = float(voltage_str)
                        self.received_vout.set(voltage)
                        print(f"Output Voltage: {voltage:.2f}V")
                    elif part.startswith("IOUT:"):
                        current_str = part.split(":")[1]
                        current = float(current_str)
                        self.received_iout.set(current)
                        print(f"Output Current: {current:.2f}mA")
                    elif part.startswith("VIN:"):
                        vin_str = part.split(":")[1]
                        vin = float(vin_str)
                        self.received_vin.set(vin)
                        print(f"Input Voltage: {vin:.2f}V")
            else:
                # Handle single value messages
                if data.startswith("VOUT:"):
                    voltage_str = data.split(":")[1]
                    voltage = float(voltage_str)
                    self.received_vout.set(voltage)
                    print(f"Output Voltage: {voltage:.2f}V")
                elif data.startswith("IOUT:"):
                    current_str = data.split(":")[1]
                    current = float(current_str)
                    self.received_iout.set(current)
                    print(f"Output Current: {current:.2f}mA")
                elif data.startswith("VIN:"):
                    vin_str = data.split(":")[1]
                    vin = float(vin_str)
                    self.received_vin.set(vin)
                    print(f"Input Voltage: {vin:.2f}V")
        except Exception as e:
            print(f"Error processing data '{data}': {e}")
    


    def update_display(self):
        self.voltage_display.config(text=f"{self.voltage.get():.2f} V")
        self.current_display.config(text=f"{int(self.current.get())} mA")
        
        # Update received values display
        self.vout_display.config(text=f"{self.received_vout.get():.2f} V")
        self.iout_display.config(text=f"{self.received_iout.get():.2f} mA")
        self.vin_display.config(text=f"{self.received_vin.get():.2f} V")
        
        # Read from serial port during each update cycle
        self.read_from_serial()
        
        self.root.after(100, self.update_display)

    def set_voltage_from_entry(self):
        try:
            val = float(self.voltage_entry.get())
            val = max(0.0, min(30.0, val))
            self.voltage.set(val)
        except:
            pass

    def set_current_from_entry(self):
        try:
            val = int(self.current_entry.get())
            val = max(0, min(5000, val))
            self.current.set(val)
        except:
            pass

    def slider_voltage_changed(self, event=None):
        self.voltage_entry.delete(0, 'end')
        self.voltage_entry.insert(0, f"{self.voltage.get():.2f}")

    def slider_current_changed(self, event=None):
        self.current_entry.delete(0, 'end')
        self.current_entry.insert(0, f"{int(self.current.get())}")

if __name__ == "__main__":
    root = ttk.Window(themename="darkly")  # You can try "cosmo", "superhero", "journal", etc.
    app = CDCControlApp(root)
    root.mainloop()
