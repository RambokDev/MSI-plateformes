import minimalmodbus
import serial
import binascii
import time
import robotiqGripper as rq

instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 9, debug = True)

# instrument.serial.port                     # this is the serial port name
instrument.serial.baudrate = 115200         # Baud
# instrument.serial.bytesize = 8
# instrument.serial.parity   = serial.PARITY_NONE
# instrument.serial.stopbits = 1
# instrument.serial.timeout  = 0.2          # seconds
# instrument.address = 9                         # this is the slave address number
# instrument.mode = minimalmodbus.MODE_RTU   # rtu or ascii mode
# instrument.clear_buffers_before_each_transaction = True
#
# print(instrument)

# instrument.write_registers(1000,[0,0,0]) # Reset the gripper
# instrument.write_registers(1000,[256,0,0]) #Activate the gripper


myGripper = rq.RobotiqGripper(portname='/dev/ttyUSB0',slaveaddress=9)
myGripper.activate()
# myGripper.openGripper(50,255)
# myGripper.closeGripper(50,255)
# time.sleep(5)
myGripper.openGripper(255,255)
myGripper.closeGripper(50,255)
# time.sleep(2)
# myGripper.goTo(0)
# time.sleep(1)
# myGripper.goTo(125)
# time.sleep(1)
# myGripper.goTo(255)
#
# def main():
#     print("1 ==== OPEN")
#     print("2 ==== CLOSE")
#
#
#     myGripper = rq.RobotiqGripper(portname='/dev/ttyUSB0',slaveaddress=9)
#     myGripper.activate()
#
#
#
#     while True:
#         user_input = input("Enter a number (or 'q' to quit): ")
#         if user_input == 'q':
#             break  # Exit the loop if the user enters 'q'
#
#         # Perform some operation with the user input
#         choice = int(user_input)
#         if choice == 1:
#             myGripper.openGripper(1,1)
#             myGripper.closeGripper(1,1)
#
#         elif choice == 2:
#             myGripper.openGripper(255,255)
#             myGripper.closeGripper(255,255)
#
#         else:
#             myGripper.goTo(0)
#             time.sleep(1)
#             myGripper.goTo(125)
#             time.sleep(1)
#             myGripper.goTo(255)
#
# if __name__ == '__main__':
#     main()
