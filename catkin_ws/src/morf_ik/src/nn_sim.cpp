// https://leenissen.dk/fann/html/files2/gettingstarted-txt.html
// https://github.com/MathiasThor/Genetic_ANN_Ludo_player/blob/master/ann_code/main.cpp
// https://leenissen.dk/fann/html/files2/advancedusage-txt.html

#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <random>
#include <fstream>
#include <vector>
#include <string>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include "floatfann.h"
#include "fann_cpp.h"

#include <sys/stat.h>


#include "../include/coordinates.hpp"
#include "../include/controller.hpp"
using namespace coords;
using namespace controller;


int main(int argc, char **argv)
{
    const unsigned int num_input = 3;
    const unsigned int num_output = 3;
    const unsigned int num_layers = 4;
    const unsigned int num_neurons_hidden = 10;
    const unsigned int max_epochs = 50000;
    const unsigned int epochs_between_reports = 1;
    const float desired_error = (const float) 0.02;
    const float learning_rate = (const float) 0.1;

    const uint layers[num_layers] = {num_input, 
                                    num_neurons_hidden, num_neurons_hidden,
                                    num_output};

    int div=5, divZ=div*2;
    int prevj=-1, prevk=-1, prevl=-1;

    std::string train_name = "batch_02_10_01_50000_02_09/"; // naming: algorithm_numHiddenLayers_numNeuronsHidden_learningRate_maxEpochs_error_scaling.dat
    std::string res_name = "./neural_networks/results_" + std::to_string(div) + "div_babbling/" + std::string(train_name);
    std::string dat_name = "./neural_networks/data_" + std::to_string(div) + "div_babbling/" + std::string(train_name);


    for(int j=0; j<=div; j++)
    {
        for(int k=0; k<=div; k++)
        {
            for(int l=0; l<=divZ; l++)
            {
                std::string filename = "./babbling_data/" + std::to_string(div) + "div/"+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".data");
                std::string filename_vali = "./babbling_data/" + std::to_string(div) + "div/"+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".data");

                if (FILE *file = fopen(filename.c_str(), "r")) 
                {
                    fclose(file);

                    struct fann *ann;

                    ann = fann_create_standard_array(num_layers, layers);
                    
                    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
                    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC); // FANN_SIGMOID_SYMMETRIC

                    fann_set_training_algorithm(ann, FANN_TRAIN_BATCH); //FANN_TRAIN_INCREMENTAL, FANN_TRAIN_BATCH, FANN_TRAIN_RPROP, FANN_TRAIN_QUICKPROP, FANN_TRAIN_SARPROP
                    fann_set_learning_rate(ann, learning_rate);
                    fann_randomize_weights(ann, -1, 1);

                    // if(prevj==-1 && prevk==-1 && prevl==-1)
                    // {
                    //     ann = fann_create_standard_array(num_layers, layers);
                        
                    //     fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
                    //     fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC); // FANN_SIGMOID_SYMMETRIC

                    //     fann_set_training_algorithm(ann, FANN_TRAIN_BATCH); //FANN_TRAIN_INCREMENTAL, FANN_TRAIN_BATCH, FANN_TRAIN_RPROP, FANN_TRAIN_QUICKPROP, FANN_TRAIN_SARPROP
                    //     fann_set_learning_rate(ann, learning_rate);
                    //     fann_randomize_weights(ann, -1, 1);
                    // }
                    // else
                    // {
                    //     std::string prev_ann = dat_name+std::to_string(prevj)+std::to_string(prevk)+std::to_string(prevl)+std::string(".net");
                    //     ann = fann_create_from_file(prev_ann.c_str());
                    // }

                    fann_train_data *train = fann_read_train_from_file(filename.c_str());
                    fann_train_data *vali = fann_read_train_from_file(filename_vali.c_str());

                    fann_set_input_scaling_params(ann, train, -0.9, 0.9);
                    fann_set_output_scaling_params(ann, train, -0.9, 0.9);
                    fann_scale_train(ann, train);
                    fann_scale_train(ann, vali);

                    // struct fann *ann = fann_create_from_file("data/ann.net");
                    // fann_train_data *train = fann_read_train_from_file("data/train.data");
                    // fann_train_data *vali = fann_read_train_from_file("data/vali.data");
                    // fann_scale_train(ann, train);
                    // fann_scale_train(ann, vali);

                    

                    std::ofstream test_log;
                    mkdir(res_name.c_str(),0777);
                    std::string logname = res_name+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".dat");
                    test_log.open(logname); 

                    double error = 0;
                    for(int i = 1 ; i <= max_epochs ; i++) {
                      fann_shuffle_train_data(train); // "This is recommended for incremental training, while it has no influence during batch training."
                      error = fann_train_epoch(ann, train);
                      printf("Section: %d%d%d \t Epoch: %d \t Training Error: %f", j, k, l, i, error); 

                      //fann_reset_MSE(ann);
                      mkdir(dat_name.c_str(),0777);
                      std::string ann_name = dat_name+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".net");
                      fann_save(ann, ann_name.c_str());
                      struct fann *ann_train = fann_create_from_file(ann_name.c_str());

                      for(int i = 0 ; i != fann_length_train_data(vali) ; i++ ) {
                        fann_test(ann_train, fann_get_train_input(vali, i), fann_get_train_output(vali, i));
                      }
                      test_log << i << "\t" << error << "\t" << fann_get_MSE(ann_train) << std::endl;
                      std::cout << "  \tValidation Error: " << fann_get_MSE(ann_train) << std::endl;

                      if ( error < desired_error) {
                        break;
                      }
                    }

                    //fann_save(ann, "data/ann.net");
                    fann_destroy(ann);
                    test_log.close();

                    prevj = j;
                    prevk = k;
                    prevl = l;
                }

            }
        }
    }
    // for(int j=0; j<div; j++)
    // {
    //     for(int k=0; k<div; k++)
    //     {
    //         for(int l=0; l<div; l++)
    //         {
    //           struct fann *ann = fann_create_standard_array(num_layers, layers);

    //           fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
    //           fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC); // FANN_SIGMOID_SYMMETRIC

    //           fann_set_training_algorithm(ann, FANN_TRAIN_BATCH); //FANN_TRAIN_INCREMENTAL, FANN_TRAIN_BATCH, FANN_TRAIN_RPROP, FANN_TRAIN_QUICKPROP, FANN_TRAIN_SARPROP
    //           fann_set_learning_rate(ann, learning_rate);
    //           fann_randomize_weights(ann, -1, 1);

    //           std::string filename = "./babbling_data/5div/"+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".data");
    //           std::string filename_vali = "./babbling_data/5div/"+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".data");

    //           if (FILE *file = fopen(filename.c_str(), "r")) 
    //           {
    //               fclose(file);

    //               fann_train_data *train = fann_read_train_from_file(filename.c_str());
    //               fann_train_data *vali = fann_read_train_from_file(filename_vali.c_str());

    //               fann_set_input_scaling_params(ann, train, -0.9, 0.9);
    //               fann_set_output_scaling_params(ann, train, -0.9, 0.9);
    //               fann_scale_train(ann, train);
    //               fann_scale_train(ann, vali);

    //               // struct fann *ann = fann_create_from_file("data/ann.net");
    //               // fann_train_data *train = fann_read_train_from_file("data/train.data");
    //               // fann_train_data *vali = fann_read_train_from_file("data/vali.data");
    //               // fann_scale_train(ann, train);
    //               // fann_scale_train(ann, vali);

    //               std::string train_name = "batch_01_05_01_50000_04_09/"; // naming: algorithm_numHiddenLayers_numNeuronsHidden_learningRate_maxEpochs_error_scaling.dat
    //               std::string res_name = "./neural_networks/results_5div_babbling/" + std::string(train_name);
    //               std::string dat_name = "./neural_networks/data_5div_babbling/" + std::string(train_name);

    //               std::ofstream test_log;
    //               mkdir(res_name.c_str(),0777);
    //               std::string logname = res_name+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".dat");
    //               test_log.open(logname); 

    //               double error = 0;
    //               for(int i = 1 ; i <= max_epochs ; i++) {
    //                 fann_shuffle_train_data(train); // "This is recommended for incremental training, while it has no influence during batch training."
    //                 error = fann_train_epoch(ann, train);
    //                 printf("Section: %d%d%d \t Epoch: %d \t Training Error: %f",j, k, l, i, error); 

    //                 //fann_reset_MSE(ann);
    //                 mkdir(dat_name.c_str(),0777);
    //                 std::string ann_name = dat_name+std::to_string(j)+std::to_string(k)+std::to_string(l)+std::string(".net");
    //                 fann_save(ann, ann_name.c_str());
    //                 struct fann *ann_train = fann_create_from_file(ann_name.c_str());

    //                 for(int i = 0 ; i != fann_length_train_data(vali) ; i++ ) {
    //                   fann_test(ann_train, fann_get_train_input(vali, i), fann_get_train_output(vali, i));
    //                 }
    //                 test_log << i << "\t" << error << "\t" << fann_get_MSE(ann_train) << std::endl;
    //                 std::cout << "  \tValidation Error: " << fann_get_MSE(ann_train) << std::endl;

    //                 if ( error < desired_error) {
    //                   break;
    //                 }
    //               }

    //               //fann_save(ann, "data/ann.net");
    //               fann_destroy(ann);
    //               test_log.close();
    //           } 
    //         }
    //     }
    // }

    // struct fann *ann = fann_create_standard_array(num_layers, layers);

    // fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
    // fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC); // FANN_SIGMOID_SYMMETRIC

    // fann_set_training_algorithm(ann, FANN_TRAIN_BATCH); //FANN_TRAIN_INCREMENTAL, FANN_TRAIN_BATCH, FANN_TRAIN_RPROP, FANN_TRAIN_QUICKPROP, FANN_TRAIN_SARPROP
    // fann_set_learning_rate(ann, learning_rate);
    // fann_randomize_weights(ann, -1, 1);

    // fann_train_data *train = fann_read_train_from_file("data/train.data");
    // fann_train_data *vali = fann_read_train_from_file("data/vali.data");

    // fann_set_input_scaling_params(ann, train, -1, 1);
    // fann_set_output_scaling_params(ann, train, -1, 1);
    // fann_scale_train(ann, train);
    // fann_scale_train(ann, vali);

    // // struct fann *ann = fann_create_from_file("data/ann.net");
    // // fann_train_data *train = fann_read_train_from_file("data/train.data");
    // // fann_train_data *vali = fann_read_train_from_file("data/vali.data");
    // // fann_scale_train(ann, train);
    // // fann_scale_train(ann, vali);

    // std::ofstream test_log;
    // test_log.open ("results/batch_02_04_01_100000_03.dat"); // naming: algorithm_numHiddenLayers_numNeuronsHidden_learningRate_maxEpochs_error.dat

    // double error = 0;
    // for(int i = 1 ; i <= max_epochs ; i++) {
    //   fann_shuffle_train_data(train); // "This is recommended for incremental training, while it has no influence during batch training."
    //   error = fann_train_epoch(ann, train);
    //   printf("Epoch: %d \t Training Error: %f", i, error); 

    //   //fann_reset_MSE(ann);

    //   fann_save(ann, "data/ann.net");
    //   struct fann *ann_train = fann_create_from_file("data/ann.net");

    //   for(int i = 0 ; i != fann_length_train_data(vali) ; i++ ) {
    //     fann_test(ann_train, fann_get_train_input(vali, i), fann_get_train_output(vali, i));
    //   }
    //   test_log << i << "\t" << error << "\t" << fann_get_MSE(ann_train) << std::endl;
    //   std::cout << "  \tValidation Error: " << fann_get_MSE(ann_train) << std::endl;

    //   if ( error < desired_error) {
    //     break;
    //   }
    // }

    // //fann_save(ann, "data/ann.net");
    // fann_destroy(ann);
    // test_log.close();

    return 0;
}
