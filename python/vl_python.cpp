#include <time.h>
#include <sys/time.h>
#include <stdio.h>

#include <stdio.h>
#include <signal.h>
#include <string>
#include <map>

#include <json/value.h>
#include <json/reader.h>

#include "vl_driver.h"
#include "vl_config.h"
#include "vl_light.h"

#include "vl_python.h"

vl_driver* driver;


static void signal_interrupt_handler(int sig) {
    signal(sig, SIG_IGN);
    delete(driver);
    exit(0);
}

ViveLibre::ViveLibre() {
  connect();
}

ViveLibre::~ViveLibre() {
    delete(driver);
}

void ViveLibre::connect() {
    driver = new vl_driver();
    if (!driver->init_devices(0))
        return;
    signal(SIGINT, signal_interrupt_handler);
}

std::string ViveLibre::get_config() {
  char * config = vl_get_config(driver->hmd_imu_device);
  return std::string(config);
}

std::map<int, std::vector<float>> ViveLibre::pollAngles() {

    vl_lighthouse_samples * raw_light_samples = new vl_lighthouse_samples();
    
    query_fun read_hmd_light = [raw_light_samples](unsigned char *buffer, int size) {
        if (buffer[0] == VL_MSG_HMD_LIGHT) {
            vive_headset_lighthouse_pulse_report2 pkt;
            vl_msg_decode_hmd_light(&pkt, buffer, size);
            //vl_msg_print_hmd_light_csv(&pkt);
            
            for(int i = 0; i < 9; i++){
                raw_light_samples->push_back(pkt.samples[i]);
            }
        }
    };
    
        while(raw_light_samples->size() < 1000)
            hid_query(driver->hmd_light_sensor_device, read_hmd_light);

        vl_lighthouse_samples sanitized_light_samples = filter_reports(*raw_light_samples, &is_sample_valid);
        std::vector<vl_light_sample_group> pulses;
        std::vector<vl_light_sample_group> sweeps;
        std::tie(sweeps, pulses) = process_lighthouse_samples(sanitized_light_samples);
        std::map<unsigned, vl_angles> R_B = collect_readings('A', sweeps);
        
        printf("got %d readings\n", R_B.size());    
  
/*
        cv::Mat rvec, tvec;
        
        if (!raw_light_samples->empty())
            std::tie(tvec, rvec) = try_pnp(raw_light_samples, config_sensor_positions);
*/        


        std::map<int, std::vector<float>> angleResult;

        for (auto angles : R_B) {

            vl_angles a = angles.second;

            //cv::Point2f a_cv = cv::Point2f(angles.second.x[0], angles.second.y[0]);
            
            std::vector<float> angleVector = {angles.second.x[0], angles.second.y[0]};
            
            angleResult[angles.first] = angleVector;
        }

        return angleResult;
}

