
#!/home/airlab/anaconda3/envs/droneController/bin/python
import sys
from PyQt5.QtWidgets import QApplication
from DroneController import MainWindow, ConfigParser


if __name__ == '__main__':
    config = ConfigParser('param.yaml')

    app = QApplication(sys.argv)
    window = MainWindow(config)
    window.show()
    sys.exit(app.exec())