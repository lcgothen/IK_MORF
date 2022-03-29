// https://leenissen.dk/fann/html/files2/gettingstarted-txt.html
// https://github.com/MathiasThor/Genetic_ANN_Ludo_player/blob/master/ann_code/main.cpp

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <random>
#include <fstream>
#include <vector>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include "floatfann.h"
#include "fann_cpp.h"


#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;


int main(int argc, char **argv)
{
    const unsigned int num_input = 3;
    const unsigned int num_output = 3;
    const unsigned int num_layers = 4;
    const unsigned int num_neurons_hidden = 15;
    const unsigned int max_epochs = 500000;
    const unsigned int epochs_between_reports = 1;
    const float desired_error = (const float) 0.01;
    const float learning_rate = (const float) 0.2;

    struct fann *ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_neurons_hidden, num_output);

    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);

    fann_set_training_algorithm(ann, FANN_TRAIN_BATCH); //FANN_TRAIN_INCREMENTAL
    fann_set_learning_rate(ann, learning_rate);
    fann_randomize_weights(ann, -1, 1);

    // fann_train_data *train = fann_read_train_from_file("data/train.data");
    // fann_scale_train_data(train,-1,1);
    // fann_train_on_data(ann, train, max_epochs, epochs_between_reports, desired_error);

    // fann_save(ann, "data/ann.net");

    // fann_train_data *vali = fann_read_train_from_file("data/vali.data");
    // fann_scale_train_data(vali,-1,1);
    // std::cout << fann_test_data(ann, vali) << std::endl;

    fann_train_data *train = fann_read_train_from_file("data/train.data");
    fann_train_data *vali = fann_read_train_from_file("data/vali.data");

    fann_set_input_scaling_params(ann, train, -1, 1);
    fann_set_output_scaling_params(ann, train, -1, 1);
    fann_scale_train(ann, train);
    fann_scale_train(ann, vali);

    std::ofstream test_log;
    test_log.open ("results/batch_02_15_02_5000.dat"); // naming: algorithm_numHiddenLayers_numNeuronsHidden_learningRate_maxEpochs.dat

    double error = 0;
    for(int i = 1 ; i <= max_epochs ; i++) {
      error = fann_train_epoch(ann, train);
      printf("Epoch: %d \t Training Error: %f", i, error); 

      fann_reset_MSE(ann);
      for(int i = 0 ; i != fann_length_train_data(vali) ; i++ ) {
        fann_test(ann, fann_get_train_input(vali, i), fann_get_train_output(vali, i));
      }
      test_log << i << "\t" << error << "\t" << fann_get_MSE(ann) << std::endl;
      std::cout << "  \tValidation Error: " << fann_get_MSE(ann) << std::endl;

      if ( error < desired_error) {
        break;
      }
    }

    fann_save(ann, "data/ann.net");
    fann_destroy(ann);
    test_log.close();

    return 0;
}
