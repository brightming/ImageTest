
#include <math.h>
#include <iostream>
#include <set>
#include <vector>

using namespace std;


void test_split_seg_height(){
    int absolute_trap_height=125;
    float height_ratio=1.2;
    int seg_n=5;
    float base_height;
    if(height_ratio!=1){
        base_height=absolute_trap_height*1.*(1-height_ratio)/(1-pow(height_ratio,seg_n));
    }
    float H=0;
    for(int i=0;i<seg_n;i++){
        int h=base_height*pow(height_ratio,seg_n-1-i);
        H+=h;
        cout<<"i="<<i<<",h="<<h<<endl;
        if(i==seg_n-1){
            h+=(absolute_trap_height-H);
            cout<<"last segment ,h is modified to : "<<h<<endl;
        }
    }
    cout<<"total H="<<H<<",original Height="<<absolute_trap_height<<endl;
}


void test_set(){
    set<int> ms;
    vector<int> area_seq_vec;

    for(int i=0;i<4;i++){
        for(int j=0;j<i;j++){
            if(ms.find(i) == ms.end()){
                ms.insert(i);
                area_seq_vec.push_back(i);
            }else{
                cout<<i<<" already exists!"<<endl;
            }

        }
    }



    set<int>::const_iterator iter;
    for(iter = ms.begin() ; iter != ms.end() ; ++iter)
    {
        cout<<*iter<<" ";
    }
    cout<<"set count="<<ms.size()<<",vector size="<<area_seq_vec.size()<<endl;

    for(int i=0;i<area_seq_vec.size();i++){
        cout<<area_seq_vec[i]<<endl;
    }
}

int main(){

    test_set();
}
