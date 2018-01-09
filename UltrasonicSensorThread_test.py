#!/usr/bin/python
"""
Provides the test functionality for the Ultrasonic Sensor Thread
"""

import logging
import time
import SetupConsoleLogger
import UltrasonicSensorThread
import GPIOLayout

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER, logging.DEBUG)

CALLBACK_CALLED = False
DISTANCE = 0.0


def __callback_UltrasonicSensorThread(data):
    MODULE_LOGGER.critical("UltrasonicSensorThread callback: " + str(data))
    global CALLBACK_CALLED, DISTANCE
    CALLBACK_CALLED = True
    DISTANCE = data


def test_ultrasonicthread_poll(sleep_len=2):
    MODULE_LOGGER.critical("test_ultrasonicthread_poll")
    try:
        SENSOR = UltrasonicSensorThread.UltrasonicSensorThread(
            1, None, GPIOLayout.SONAR_FRONT_TX_PIN,
            GPIOLayout.SONAR_FRONT_RX_PIN, 1)
        SENSOR.start()
        time.sleep(sleep_len)
        MODULE_LOGGER.info('Read distance from thread ={0:0.2f} cm '.format(
            SENSOR.read_data()))
    except KeyboardInterrupt:
        pass
    finally:
        SENSOR.exit_now()
        SENSOR.join()
        SENSOR.__del__()


def test_ultrasonicthread_callback(sleep_len=2):
    MODULE_LOGGER.info("test_ultrasonicthread_callback")
    try:
        global CALLBACK_CALLED, DISTANCE
        CALLBACK_CALLED = False
        DISTANCE = 0.0
        SENSOR = UltrasonicSensorThread.UltrasonicSensorThread(
            1, __callback_UltrasonicSensorThread,
            GPIOLayout.SONAR_FRONT_TX_PIN, GPIOLayout.SONAR_FRONT_RX_PIN, 1)
        SENSOR.start()
        time.sleep(sleep_len)
        MODULE_LOGGER.info(
            'Read distance from callback ={0:0.2f} cm '.format(DISTANCE))
        assert CALLBACK_CALLED is True
    except KeyboardInterrupt:
        pass
    finally:
        SENSOR.exit_now()
        SENSOR.join()
        SENSOR.__del__()

        
def test_ultrasonicthread_multiplesensors(sleep_len=2):
    MODULE_LOGGER.critical("test_ultrasonicthread_multiplesensors")
    try:
        FRONT_SENSOR = UltrasonicSensorThread.UltrasonicSensorThread(
                        1, None, GPIOLayout.SONAR_FRONT_TX_PIN,
                        GPIOLayout.SONAR_FRONT_RX_PIN, 1)
        FRONT_SENSOR.start()

        RIGHT_SENSOR = UltrasonicSensorThread.UltrasonicSensorThread(
                        1, None, GPIOLayout.SONAR_RIGHT_TX_PIN,
                        GPIOLayout.SONAR_RIGHT_RX_PIN, 1)
        RIGHT_SENSOR.start()

        LEFT_SENSOR = UltrasonicSensorThread.UltrasonicSensorThread(
                        1, None, GPIOLayout.SONAR_LEFT_TX_PIN,
                        GPIOLayout.SONAR_LEFT_RX_PIN, 1)
        LEFT_SENSOR.start()
        time.sleep(sleep_len)
        MODULE_LOGGER.info('FRONT_SENSOR distance from thread ={0:0.2f} cm '.format(
            FRONT_SENSOR.read_data()))
        MODULE_LOGGER.info('RIGHT_SENSOR distance from thread ={0:0.2f} cm '.format(
            RIGHT_SENSOR.read_data()))
        MODULE_LOGGER.info('LEFT_SENSOR distance from thread ={0:0.2f} cm '.format(
            LEFT_SENSOR.read_data()))
    except KeyboardInterrupt:
        pass
    finally:
        MODULE_LOGGER.info('Starting cleanup')
        FRONT_SENSOR.exit_now()
        FRONT_SENSOR.join()
        FRONT_SENSOR.__del__()
        RIGHT_SENSOR.exit_now()
        RIGHT_SENSOR.join()
        RIGHT_SENSOR.__del__()
        LEFT_SENSOR.exit_now()
        LEFT_SENSOR.join()
        LEFT_SENSOR.__del__()
        MODULE_LOGGER.info('Cleanup complete')


if __name__ == "__main__":
    test_ultrasonicthread_poll(3)
    test_ultrasonicthread_callback(3)
    test_ultrasonicthread_multiplesensors(3)
