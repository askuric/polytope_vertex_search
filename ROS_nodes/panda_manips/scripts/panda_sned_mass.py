#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {desired_mass}, {k_p}, {k_i}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client_panda_table = dynamic_reconfigure.client.Client("/panda_table/dynamic_reconfigure_desired_mass_param_node", timeout=30, config_callback=callback)
    client_panda_box = dynamic_reconfigure.client.Client("/panda_box/dynamic_reconfigure_desired_mass_param_node", timeout=30, config_callback=callback)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        client_panda_table.update_configuration({"desired_mass":2, "k_p":0, "k_i":0})
        client_panda_box.update_configuration({"desired_mass":2, "k_p":0, "k_i":0})
        r.sleep()