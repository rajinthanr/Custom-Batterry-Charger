import ttkbootstrap as ttk
from ttkbootstrap.constants import *
import serial
import serial.tools.list_ports

class CDCControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Voltage & Current Control")
        self.root.geometry("620x550")
        self.ser = None

        self.voltage = ttk.DoubleVar(value=5.00)
        self.current = ttk.IntVar(value=500)
        self.port_var = ttk.StringVar()

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

    def update_display(self):
        self.voltage_display.config(text=f"{self.voltage.get():.2f} V")
        self.current_display.config(text=f"{int(self.current.get())} mA")
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
