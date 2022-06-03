import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sd_interfaces.msg import DroneMsgOut


import tkinter
import time


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("sd_visualiser")
        self.declare_parameter("num_drones", 4)
        self.num_drones = self.get_parameter(
            "num_drones").get_parameter_value().integer_value

        self.w = 300
        self.h = 500
        self.a = 20
        self.s = 20
        self.drone_headline = []
        self.drone_x = []
        self.drone_y = []
        self.drone_z = []
        self.drone_battery = []
        self.drone_mode = []
        self.drone_flight_mode = []

        self.rect = []

        # tkinter
        wh = self.s + self.num_drones * (self.w + self.a)
        self.root = tkinter.Tk()
        self.root.geometry(f"{str(wh)}x700")
        self.canvas = tkinter.Canvas(
            self.root, bg="white", width=wh, height=700)
        self.canvas.pack()

        for i in range(self.num_drones):

            self.rect.append(self.canvas.create_rectangle(
                self.s + i*(self.w+self.a), self.s, self.s+self.w + i * (self.w+self.a), self.s+self.h))
            self.drone_headline.append(self.canvas.create_text(
                (self.s + 150 + i*(self.w+self.a), self.s+20), text=f"Drone_{i}", font=('Helvetica', 12, 'bold')))

            self.drone_x.append(self.canvas.create_text(
                (self.s + 150 + i*(self.w+self.a), self.s+60), text=f"X: ", font=('Helvetica', 9, )))

            self.drone_y.append(self.canvas.create_text(
                (self.s + 150 + i*(self.w+self.a), self.s+100), text=f"Y: ", font=('Helvetica', 9, )))

            self.drone_z.append(self.canvas.create_text(
                (self.s + 150 + i*(self.w+self.a), self.s+140), text=f"Z: ", font=('Helvetica', 9, )))

            self.drone_battery.append(self.canvas.create_text(
                (self.s + 150 + i*(self.w+self.a), self.s+180), text=f"Battery: ", font=('Helvetica', 9, )))

            self.drone_mode.append(self.canvas.create_text(
                (self.s + 150 + i*(self.w+self.a), self.s+220), text=f"Drone Mode : ", font=('Helvetica', 9, )))

            self.drone_flight_mode.append(self.canvas.create_text(
                (self.s + 150 + i*(self.w+self.a), self.s+260), text=f"Flight Mode: ", font=('Helvetica', 9, )))

        self.subscription = self.create_subscription(
            DroneMsgOut,
            "/drones/infos",
            self.listener_callback,
            10)

        self.root.update()

        self.msg = [None for i in range(self.num_drones)]
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        i = msg.drone_header.drone_id
        self.msg[i] = msg
        self.update_canvas()

    def update_canvas(self):

        for i in range(self.num_drones):
            self.canvas.itemconfigure(
                self.drone_headline[i], text=f"Drone_{i}")
            if self.msg[i]:
                self.canvas.itemconfigure(
                    self.drone_x[i], text=f"X: {round(self.msg[i].drone_header.pos.x, 1)}")
                self.canvas.itemconfigure(
                    self.drone_y[i], text=f"Y: {round(self.msg[i].drone_header.pos.y, 1)}")
                self.canvas.itemconfigure(
                    self.drone_z[i], text=f"Z: {round(self.msg[i].drone_header.pos.z, 1)}")
                self.canvas.itemconfigure(
                    self.drone_battery[i], text=f"battery: {round(self.msg[i].drone_header.battery, 1)}")

                mode = self.msg[i].drone_header.drone_mode
                mode_text = None
                color = None
                if mode == 0:
                    mode_text = "Not Ready"
                    color = "red"
                elif mode == 1:
                    mode_text = "Ready"
                    color = "yellow"
                elif mode == 2:
                    mode_text = "Flying"
                    color = "green"

                self.canvas.itemconfigure(
                    self.drone_mode[i], text=f"Drone Mode: {mode_text}")

                self.canvas.itemconfigure(
                    self.rect[i], fill=color)

                fmode = self.msg[i].drone_header.flight_mode
                fmode_text = None
                if fmode == 0:
                    fmode_text = "Landed"
                elif fmode == 1:
                    fmode_text = "Starting"
                elif fmode == 2:
                    fmode_text = "Landing"
                elif fmode == 3:
                    fmode_text = "Flying"

                self.canvas.itemconfigure(
                    self.drone_flight_mode[i], text=f"Flight Mode: {fmode_text}")

        self.root.update()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()