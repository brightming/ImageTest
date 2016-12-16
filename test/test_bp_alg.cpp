

#include <iostream>
#include <vector>
#include <memory>
#include <math.h>

#include <opencv2/opencv.hpp>



using namespace std;
using namespace cv;

float sigmoid(float input){
    float ret=1.0/(1+exp(input*-1));
    return ret;
}

float derivative_sigmoid(float input){
    float ret=sigmoid(input)*(1-sigmoid(input));
    return ret;
}

float relu(float input){
   if(input>0){
       return input;
   }else{
       return 0;
   }
}
float derivative_relu(float input){

    if(input>0){
        return 1;
    }else{
        return 0;
    }
}


enum NeuronType{

    Input,
    Middle,
    Output
};


class Neuron{
public:
    int idx;
    float input;
    float output;
    vector<shared_ptr<Neuron> > output_neuron;
    vector<float> weights;
    NeuronType type;
    float delta;
public:
    Neuron(NeuronType tp,int id){
        this->type=tp;
        this->idx=id;
    }
    void AddConnectedNeuron(shared_ptr<Neuron> next,float weight){
        output_neuron.push_back(next);
        weights.push_back(weight);
    }
    void Print(){
        std::cout<<"id="<<idx<<",input="<<input<<",output="<<output<<",delta="<<delta<<endl;

    }
};


class Layer{
public:
    int lay_idx;
    NeuronType type;
    vector<shared_ptr<Neuron> > neurons;
    Layer* next_layer;

public:
    Layer(int idx,int neuron_cnt,NeuronType tp){
        this->lay_idx=idx;
        this->type=tp;
        for(int i=0;i<neuron_cnt;i++){
            shared_ptr<Neuron> neu(new Neuron(tp,i));
            neurons.push_back(neu);
        }
    }

    int GetNeuronsSize(){
        return neurons.size();
    }

    shared_ptr<Neuron> GetNeuron(int idx){
        if(idx<neurons.size()){
            return neurons[idx];
        }else{
            return NULL;
        }
    }

    void Print(){
        std::cout<<"\n-----------------------print layer:"<<lay_idx<<" begin---------------------"<<endl;
        for(shared_ptr<Neuron> n:neurons){
            n->Print();
        }
        std::cout<<"-----------------------print layer:"<<lay_idx<<" end---------------------\n"<<endl;
    }

    int GetMaxOutputIdx(){
        int idx=0;
        shared_ptr<Neuron> neu=neurons[0];
        float val=neu->output;
        for(int i=1;i<neurons.size();i++){
            if(val<neurons[i]->output){
                val=neurons[i]->output;
                idx=i;
            }
        }
        return idx;
    }

    /**
     * @brief Forward
     * @param data
     * 一行样本，数量与input单元数量相同
     * @param expected_output
     * 一行期望输出，数量与输出单元数量相同
     */
    void Forward(cv::Mat &data,cv::Mat& expected_output){
        //产生本层的各单元的输出
        for(int i=0;i<neurons.size();i++){
            if(type==Input){
                neurons[i]->output=neurons[i]->input=data.at<float>(0,i);
            }else{
                neurons[i]->output=sigmoid(neurons[i]->input);
            }

            //因为是全连接，所以每个单元连接的下一个单元都是一样的
            //将下一层的输入修改为0
            if(i==0){
                for(int j=0;j<neurons[i]->output_neuron.size();j++){
                    neurons[i]->output_neuron[j]->input=0;
                }
            }
        }

        //产生对下一层单元的输入
        if(type!=Output){
            for(int i=0;i<neurons.size();i++){
                for(int j=0;j<neurons[i]->output_neuron.size();j++){
                    neurons[i]->output_neuron[j]->input+=neurons[i]->weights[j]*neurons[i]->output;//权重*输出
                }
            }
        }else if(!expected_output.empty()){//有提供label
            //output层的单元，计算误差项
            if(neurons.size()!=expected_output.cols){
                std::cerr<<"layer forward wrong!output neurons' size("<<neurons.size()<<") expected to equal to the output size("<<expected_output.size()<<")!"<<std::endl;
            }else{
                for(int i=0;i<neurons.size();i++){
                    neurons[i]->delta=
                            (neurons[i]->output-expected_output.at<float>(0,i))
                            *
                            derivative_sigmoid(neurons[i]->input);
                }
            }

        }

    }


    /**
     * @brief Backward
     * 计算delta
     */
    void Backward(){
        if(this->type==Output){
            return;
        }
        for(shared_ptr<Neuron> neu: neurons){
            neu->delta=0;
            for(int i=0;i<neu->output_neuron.size();i++){
                shared_ptr<Neuron> next_lay_neu=neu->output_neuron[i];
                float weight=neu->weights[i];
                neu->delta+=next_lay_neu->delta*weight;
            }
            neu->delta*=derivative_sigmoid(neu->output);
        }

    }

    /**
     * @brief UpdateWeights
     * 通过上一层，来更新与下一层的连接的权重。
     *
     */
    void UpdateWeights(){
        if(this->type==Output){
            return;
        }
        for(shared_ptr<Neuron> neu: neurons){
            for(int i=0;i<neu->output_neuron.size();i++){
                shared_ptr<Neuron> next_lay_neu=neu->output_neuron[i];
                //nita固定取0.1
                neu->weights[i]-=0.0025*next_lay_neu->delta*neu->output;//w_i,j=w_i,j-delta_j*output_i;
            }
        }
    }
};


class Network{
public:
    vector<shared_ptr<Layer> > layers;

    void initialize_layers(vector<int> layer_cnt){

        layers.resize(0);

        if(layer_cnt.size()<2){
            return ;
        }
        for(int i=0;i<layer_cnt.size();i++){
            NeuronType tp=Middle;
            if(i==0){
                tp=Input;
            }else if(i==layer_cnt.size()-1){
                tp=Output;
            }

            shared_ptr<Layer> layer(new Layer(i,layer_cnt[i],tp));
            layers.push_back(layer);
        }

        //layer之间的neuron建立全连接
        for(int i=0;i<layer_cnt.size()-1;i++){

            for(int k=0;k<layers[i]->GetNeuronsSize();k++){
                shared_ptr<Neuron> pre_neu=layers[i]->GetNeuron(k);
                for(int m=0;m<layers[i+1]->GetNeuronsSize();m++){
                    shared_ptr<Neuron> next_neu=layers[i+1]->GetNeuron(m);
                    pre_neu->AddConnectedNeuron(next_neu,0.1*pow(-1,m));//随机权重值
                }
            }
        }
    }

    void Train(Mat& data,Mat& labels,int iter_cnt=100){
        //验证数据与网络的shape
        int input_neu_cnt=layers[0]->neurons.size();
        int output_neu_cnt=layers[layers.size()-1]->neurons.size();
        if(data.cols!=input_neu_cnt){
            std::cerr<<"data size("<<data.cols<<") does'nt match input layer's size("<<input_neu_cnt<<")!"<<std::endl;
            return;
        }

        if(labels.cols!=output_neu_cnt){
            std::cerr<<"labels size("<<labels.cols<<") does'nt match output layer's size("<<output_neu_cnt<<")!"<<std::endl;
            return;
        }


        int total_cnt=0;
        int right_cnt=0;
        shared_ptr<Layer> last_layer;
        for(int iter=0;iter<iter_cnt;iter++){
            total_cnt=0;
            right_cnt=0;
            for(int i=0;i<data.rows;i++){
                ++total_cnt;
                Mat row=data.row(i);
                Mat label=labels.row(i);

                for(shared_ptr<Layer> ly:layers){
                    ly->Forward(row,label);

                }
//                if(row.at<float>(0,0)==-5 && row.at<float>(0,1)==5){
//                    std::cout<<"train:data="<<row<<",label="<<label<<"\n";
//                    shared_ptr<Layer> last=layers[layers.size()-1];
//                    this->Print();
//                }

                last_layer=layers[layers.size()-1];
                int for_idx=last_layer->GetMaxOutputIdx();
                int lab_idx=0;
                for(int k=0;k<label.cols;k++){
                    if(label.at<float>(0,k)==1){
                        lab_idx=k;
                        break;
                    }
                }
                if(lab_idx==for_idx){
                    ++right_cnt;
                }else{
//                    std::cout<<"train label="<<label<<endl;
//                    last_layer->Print();
                }


                for(int j=layers.size()-1;j>=0;j--){
                    shared_ptr<Layer> ly=layers[j];
                    ly->Backward();
                }
                for(int j=layers.size()-1;j>=0;j--){
                    shared_ptr<Layer> ly=layers[j];
                    ly->UpdateWeights();
                }
            }
            std::cout<<"iter="<<iter<<",right_cnt="<<right_cnt<<",total_cnt="<<total_cnt<<",accuracy="<<1.0*right_cnt/total_cnt<<endl;
        }
    }

    int Predict(Mat& data,Mat* output_label=NULL){
        if(data.cols!=layers[0]->neurons.size()){
            std::cerr<<"size("<<data.cols<<") of data for predict does'nt match network input layer's size("<<layers[0]->neurons.size()<<") !"<<endl;
        }
        Mat empty_label;
        for(shared_ptr<Layer> ly:layers){
            ly->Forward(data,empty_label);
        }
        shared_ptr<Layer> output_layer=layers[layers.size()-1];
        float max=output_layer->neurons[0]->output;
        if(output_label!=NULL)
            output_label->at<float>(0,0)=max;
        int label=0;
        for(int i=1;i<output_layer->neurons.size();i++){
            if(output_label!=NULL)
                output_label->at<float>(0,i)=output_layer->neurons[i]->output;
            if(max<output_layer->neurons[i]->output){
                max=output_layer->neurons[i]->output;
                label=i;
            }
        }
        return label;
    }

    void Print(){
        for(shared_ptr<Layer> ly:layers){
            ly->Print();
        }
    }
};






int main(){

    //    float input=10.01;
    //    float ret=sigmoid(input);
    //    std::cout<<"input="<<input<<",sigmoid="<<ret<<std::endl;

    //    ret=devirate_sigmoid(input);
    //    std::cout<<"input="<<input<<",devirate_sigmoid="<<ret<<std::endl;


    vector<int> layer_cnt;
    layer_cnt.push_back(2);
//    layer_cnt.push_back(3);
    layer_cnt.push_back(5);
//    layer_cnt.push_back(16);
    layer_cnt.push_back(2);

    Network net;
    net.initialize_layers(layer_cnt);

    Mat inputs=Mat::zeros(400,2,CV_32FC1);
    Mat labels=Mat::zeros(400,2,CV_32FC1);

    int row_idx=0;
    for(int i=0;i<10;i++){
        for(int j=0;j<10;j++){
            inputs.at<float>(row_idx,0)=i;
            inputs.at<float>(row_idx,1)=j;

//            labels.at<float>(row_idx,0)=1;
//            labels.at<float>(row_idx,1)=0;
//            labels.at<float>(row_idx,2)=0;
//            labels.at<float>(row_idx,3)=0;

            if(i>j){

                labels.at<float>(row_idx,0)=1;
                labels.at<float>(row_idx,1)=0;
            }else{
                labels.at<float>(row_idx,0)=0;
                labels.at<float>(row_idx,1)=1;
            }

            row_idx++;
        }

        for(int j=-10;j<0;j++){
            inputs.at<float>(row_idx,0)=i;
            inputs.at<float>(row_idx,1)=j;

//            labels.at<float>(row_idx,0)=1;
//            labels.at<float>(row_idx,1)=0;
//            labels.at<float>(row_idx,2)=0;
//            labels.at<float>(row_idx,3)=1;

            if(i>j){

                labels.at<float>(row_idx,0)=1;
                labels.at<float>(row_idx,1)=0;
            }else{
                labels.at<float>(row_idx,0)=0;
                labels.at<float>(row_idx,1)=1;
            }

            row_idx++;
        }

    }
    for(int i=-10;i<0;i++){
        for(int j=0;j<10;j++){
            inputs.at<float>(row_idx,0)=i;
            inputs.at<float>(row_idx,1)=j;

//            labels.at<float>(row_idx,0)=0;
//            labels.at<float>(row_idx,1)=1;
//            labels.at<float>(row_idx,2)=0;
//            labels.at<float>(row_idx,3)=0;

            if(i>j){

                labels.at<float>(row_idx,0)=1;
                labels.at<float>(row_idx,1)=0;
            }else{
                labels.at<float>(row_idx,0)=0;
                labels.at<float>(row_idx,1)=1;
            }

            row_idx++;
        }
        for(int j=-10;j<0;j++){
            inputs.at<float>(row_idx,0)=i;
            inputs.at<float>(row_idx,1)=j;

//            labels.at<float>(row_idx,0)=0;
//            labels.at<float>(row_idx,1)=1;
//            labels.at<float>(row_idx,2)=1;
//            labels.at<float>(row_idx,3)=0;

            if(i>j){

                labels.at<float>(row_idx,0)=1;
                labels.at<float>(row_idx,1)=0;
            }else{
                labels.at<float>(row_idx,0)=0;
                labels.at<float>(row_idx,1)=1;
            }

            row_idx++;
        }





    }
    //    cout<<"row_idx="<<row_idx<<endl;



    net.Train(inputs,labels,90);

//    net.Print();

    cout<<"--------begin to predict--------------"<<endl;
    Mat test_data=Mat::zeros(15,2,CV_32FC1);
    row_idx=0;
    for(int i=50;i<55;i++){
        for(int j=51;j<55;j++){
            test_data.at<float>(row_idx,0)=i*pow(-1,row_idx);
            test_data.at<float>(row_idx,1)=j;
            ++row_idx;
        }
    }

    Mat pred_lab=Mat::zeros(1,2,CV_32FC1);
    for(int i=0;i<5;i++){
        Mat row=test_data.row(i);
        int lab=net.Predict(row,&pred_lab);
        cout<<"data="<<row<<"\npredict label="<<pred_lab<<endl;
        net.Print();
    }



//    cv::Mat mat1=Mat(2,2,CV_32FC1);
//    cv::Mat mat2=Mat(2,2,CV_32FC1);
//    mat1.at<float>(0,0) = -1.0f;
//    mat1.at<float>(0,1) = 2.0f;
//    mat1.at<float>(1,0) = -3.0f;
//    mat1.at<float>(1,1) = 400.0f;
//    // 对于这种小矩阵，还有更简单的赋值方式，找时间再改
//    cout<<"Mat 1:"<<endl;
//    cout<<mat1<<endl;

//    normalize(mat1,mat2,1.0,-1.0,NORM_MINMAX);
//    cout<<"Mat 2:"<<endl;
//    cout<<mat2<<endl;



    return 0;
}
