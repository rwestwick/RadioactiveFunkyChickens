#!/usr/bin/python"""Sets the logger to use a console"""import loggingdef setup_console_logger(logger_name):    """    Setup a console based logger    :param logger_name: The logger to attach to    :return: N/A    """    logger_name.setLevel(logging.DEBUG)    console_handler = logging.StreamHandler()    console_handler.setLevel(logging.INFO)    formatter = logging.Formatter(        '%(asctime)s - %(name)25s - %(levelname)s - %(message)s')    console_handler.setFormatter(formatter)    logger_name.addHandler(console_handler)