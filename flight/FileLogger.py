"""
Contains the FileLogger class that can be used to log Crazyflie data to an external file.
Author: Sven Pfeiffer, MAVLab
"""

import time
import json

from cflib.crazyflie.log import LogConfig


class FileLogger:
    """
    This class allows to add logging configurations to the crazyflie and write the results
    to a logfile. Logging configurations should be defined in a seperate file 'logcfg.json'
    and can then be added to the logger by name. External data (e.g. optitrack) can be added as
    external config.
    """

    def __init__(self, crazyflie, configName, fileName):
        """ Initialize and run the example with the specified link_uri """
        self._cf = crazyflie
        self.is_connected = False

        # import log configs from logcfg.json
        with open(configName) as json_config_file:
            self._cfg_defs = json.load(json_config_file)

        # list of enabled configurations
        self._enabled_configs = []
        # dictionary to hold data from callbacks
        self._data_dict = {}

        # running LogConfigs
        self._lg_conf = (
            []
        )  # can we scrap this? or do we need to be able to access these?

        # open logfile to write to
        self._logfile = open(fileName, "w")

    def __del__(self):
        self._logfile.close()

    def start(self):
        """ Commits the logging configurations and adds them to the 
        Crazyflie. Call AFTER the cf is connected."""
        if not self._cf.is_connected():
            print("Could not start logging, crazyflie not connected")
        else:
            self.is_connected = True
            self._open_log_file()
            # add log configs to cf
            counter = 0
            for cfg_name in self._enabled_configs:
                cfg = self._cfg_defs[cfg_name]
                if cfg["type"] == "CF":
                    self._add_cf_log_config(cfg_name, counter)
                    counter = counter + 1
                else:
                    print('Log config "{}" added'.format(cfg_name))

    def enableAllConfigs(self):
        """ Enable all configs in the current logcfg file"""
        for cfg in self._cfg_defs:
            self.enableConfig(cfg)

    def enableConfig(self, cfg_name):
        """ Enable a config defined in logcfg.json"""
        if cfg_name in self._cfg_defs:
            self._enabled_configs.append(cfg_name)
            for var in self._cfg_defs[cfg_name]["variables"]:
                self._data_dict[var] = 0
        else:
            print('Could not enable config "{}". Config not found.'.format(cfg_name))

    def addConfig(self, config):
        """Defines and enables a new logconfig
        @parma[in]: config - a dictionary that defines the properties of the config. Fields:
        config["name"]: name of the configuration
        config["type"]: 'CF' (register cf callback) or 'EXT' (data will be updated using the registerData function)
        config["period"]: for CF callbacks, frequency of data acquisition in ms
        config["variables"]: names of the variables to log
        config["headers"]: headers under which the variables appear in the logfile
        """
        self._cfg_defs[config["name"]] = config
        self._enabled_configs.append(config["name"])
        # self._external_configs.append(name)

    def registerData(self, config, data_dict):
        """Register data for an external logconfig. Data dict must contain the fields that
        correspond to variables of config
        """
        if config in self._enabled_configs:
            for key, value in data_dict.items():
                if key in self._cfg_defs[config]["variables"]:
                    self._data_dict[key] = value
                else:
                    print(
                        'Could not register data for variable "{}" in config "{}": Variable does not exist',
                        key,
                        config,
                    )
        else:
            print('Could not register data for config "{}": Config not active', config)

    def _add_cf_log_config(self, cfg_name, cfg_id):
        config = self._cfg_defs[cfg_name]
        self._lg_conf.append(
            LogConfig(name=config["name"], period_in_ms=config["period"])
        )

        for var in config["variables"]:
            self._lg_conf[cfg_id].add_variable(var, "float")

        try:
            self._cf.log.add_config(self._lg_conf[cfg_id])
            self._lg_conf[cfg_id].data_received_cb.add_callback(self._log_cb)
            # add the write file callback to the first log config registered
            if cfg_id == 0:
                self._lg_conf[cfg_id].data_received_cb.add_callback(
                    self._log_cb_write_file
                )
            self._lg_conf[cfg_id].error_cb.add_callback(self._log_error)
            self._lg_conf[cfg_id].start()
            print('Log config "{}" added'.format(cfg_name))
        except KeyError as e:
            print(
                "Could not start log configuration,"
                "{} not found in TOC".format(str(e))
            )
        except AttributeError:
            print("Could not add Distance log config, bad configuration.")

    def _log_cb(self, timestamp, data, logconf):
        for key, value in data.items():
            self._data_dict[key] = value

    def _log_error(self, logconf, msg):
        print("Error when logging %s: %s" % (logconf.name, msg))

    def _log_cb_write_file(self, timestamp, data, logconf):
        self._write_out_log_data(timestamp)

    def _open_log_file(self):
        self._logfile.write("timeTick")

        for cfg in self._enabled_configs:
            if cfg in self._cfg_defs:
                for header in self._cfg_defs[cfg]["headers"]:
                    self._logfile.write(", ")
                    self._logfile.write(header)

        self._logfile.write("\n")

    # Function to write the log data to file
    def _write_out_log_data(self, timetick):
        if self.is_connected:
            self._logfile.write("{}".format(timetick))

            for cfg in self._enabled_configs:
                for var in self._cfg_defs[cfg]["variables"]:
                    self._logfile.write(", {}".format(self._data_dict[var]))

            self._logfile.write("\n")
