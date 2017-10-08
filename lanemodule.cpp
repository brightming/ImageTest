
/*TODO
 * improve edge linking
 * remove blobs whose axis direction doesnt point towards vanishing pt
 * Parallelisation
 * lane prediction
*/

#include "lane/lanemodule.h"
#include "algorithm/lof.h"
#include <glog/logging.h>

namespace gumh{

bool candidate_segs_num_greator(CandidateGroup& a,CandidateGroup& b){
    return a.lane_marks.size()>b.lane_marks.size();
}

bool candidate_segs_loss_less(CandidateGroup& a,CandidateGroup& b){
    return a.loss<b.loss;
}

bool cmp_good_pt_less(LaneMarkingSeg *a,LaneMarkingSeg* b){
    return a->good_pt_cnt<b->good_pt_cnt;
}

LaneFilter::LaneFilter(Mat startFrame)
{
    //currFrame = startFrame;                                    //if image has to be processed at original size

    currFrame = Mat(720,1280,CV_8UC1,0.0);                        //initialised the image size to 320x480
    resize(startFrame, currFrame, currFrame.size());             // resize the input to required size

    temp      = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores possible lane markings
    temp2     = Mat(currFrame.rows, currFrame.cols, CV_8UC1,0.0);//stores finally selected lane marks

    vanishingPt    = currFrame.rows/3;                           //for simplicity right now
    ROIrows        = currFrame.rows - vanishingPt;               //rows in region of interest
    minSize        = 0.00005 * (currFrame.cols*currFrame.rows);  //min size of any region to be selected as lane
    maxLaneWidth   = 0.025 * currFrame.cols;                     //approximate max lane width based on image size
    smallLaneArea  = 7 * minSize;
    longLane       = 0.3 * currFrame.rows;
    ratio          = 4;

    //these mark the possible ROI for vertical lane segments and to filter vehicle glare
    vertical_left  = 2*currFrame.cols/5;
    vertical_right = 3*currFrame.cols/5;
    vertical_top   = 2*currFrame.rows/3;

//    namedWindow("lane",2);
    namedWindow("midstep", 2);
//    namedWindow("currframe", 2);
    namedWindow("laneBlobs",2);

    //    getLane();
}

void LaneFilter::updateSensitivity()
{
    int total=0, average =0;
    for(int i= vanishingPt; i<currFrame.rows; i++)
        for(int j= 0 ; j<currFrame.cols; j++)
            total += currFrame.at<uchar>(i,j);
    average = total/(ROIrows*currFrame.cols);
    cout<<"average : "<<average<<endl;
}

void LaneFilter::getLane()
{
    //medianBlur(currFrame, currFrame,5 );
    // updateSensitivity();
    //ROI = bottom half
    for(int i=vanishingPt; i<currFrame.rows; i++)
        for(int j=0; j<currFrame.cols; j++)
        {
            temp.at<uchar>(i,j)    = 0;
            temp2.at<uchar>(i,j)   = 0;
        }

    //    imshow("currframe", currFrame);
    blobRemoval();
}

void LaneFilter::markLane()
{
    for(int i=vanishingPt; i<currFrame.rows; i++)
    {
        //IF COLOUR IMAGE IS GIVEN then additional check can be done
        // lane markings RGB values will be nearly same to each other(i.e without any hue)

        //min lane width is taken to be 5
        laneWidth =5+ maxLaneWidth*(i-vanishingPt)/ROIrows;
        for(int j=laneWidth; j<currFrame.cols- laneWidth; j++)
        {

            diffL = currFrame.at<uchar>(i,j) - currFrame.at<uchar>(i,j-laneWidth);
            diffR = currFrame.at<uchar>(i,j) - currFrame.at<uchar>(i,j+laneWidth);
            diff  =  diffL + diffR - abs(diffL-diffR);

            //1 right bit shifts to make it 0.5 times
            diffThreshLow = currFrame.at<uchar>(i,j)>>1;
            //diffThreshTop = 1.2*currFrame.at<uchar>(i,j);

            //both left and right differences can be made to contribute
            //at least by certain threshold (which is >0 right now)
            //total minimum Diff should be atleast more than 5 to avoid noise
            if (diffL>0 && diffR >0 && diff>5)
                if(diff>=diffThreshLow /*&& diff<= diffThreshTop*/ )
                    temp.at<uchar>(i,j)=255;
        }
    }

}

void LaneFilter::blobRemoval()
{
    markLane();

    // find all contours in the binary image
    temp.copyTo(binary_image);
    findContours(binary_image, contours,
                 hierarchy, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE);

    keep_contours.clear();
    context->segs.clear();


    // for removing invalid blobs
    if (!contours.empty())
    {
        for (size_t i=0; i<contours.size(); ++i)
        {
            //====conditions for removing contours====//

            contour_area = contourArea(contours[i]) ;

            //blob size should not be less than lower threshold
            if(contour_area > minSize)
            {
                rotated_rect    = minAreaRect(contours[i]);
                sz              = rotated_rect.size;
                bounding_width  = sz.width;
                bounding_length = sz.height;


                //openCV selects length and width based on their orientation
                //so angle needs to be adjusted accordingly
                blob_angle_deg = rotated_rect.angle;
                if (bounding_width < bounding_length)
                    blob_angle_deg = 90 + blob_angle_deg;

                //if such big line has been detected then it has to be a (curved or a normal)lane
                if(bounding_length>longLane || bounding_width >longLane)
                {
                    drawContours(currFrame, contours,i, Scalar(255), CV_FILLED, 8);
                    drawContours(temp2, contours,i, Scalar(255), CV_FILLED, 8);

                    LaneMarkingSeg seg(context,contours[i]);
                    context->segs.push_back(seg);
                    keep_contours.push_back(contours[i]);
                }

                //angle of orientation of blob should not be near horizontal or vertical
                //vertical blobs are allowed only near center-bottom region, where centre lane mark is present
                //length:width >= ratio for valid line segments
                //if area is very small then ratio limits are compensated
                else if ((blob_angle_deg <10 || blob_angle_deg >-10 ) &&
                         ((blob_angle_deg > -90 && blob_angle_deg < 90 ) ||
                          (rotated_rect.center.y > vertical_top &&
                           rotated_rect.center.x > vertical_left && rotated_rect.center.x < vertical_right)))
                {

                    if ((bounding_length/bounding_width)>=ratio || (bounding_width/bounding_length)>=ratio
                            ||(contour_area< smallLaneArea &&  ((contour_area/(bounding_width*bounding_length)) > .75) &&
                               ((bounding_length/bounding_width)>=2 || (bounding_width/bounding_length)>=2)))
                    {
                        drawContours(currFrame, contours,i, Scalar(255), CV_FILLED, 8);
                        drawContours(temp2, contours,i, Scalar(255), CV_FILLED, 8);
                        keep_contours.push_back(contours[i]);
                        LaneMarkingSeg seg(context,contours[i]);
                        context->segs.push_back(seg);
                    }
                }
            }
        }
    }
    imshow("midstep", temp);
    imshow("laneBlobs", temp2);
//    imshow("lane",currFrame);

}


Mat LaneFilter::nextFrame(Mat &nxt)
{
    //currFrame = nxt;                        //if processing is to be done at original size

    resize(nxt ,currFrame, currFrame.size()); //resizing the input image for faster processing
    if(currFrame.channels()>1){
        cv::cvtColor(currFrame,currFrame,CV_BGR2GRAY);
    }
    getLane();

    return temp2;
}

Mat LaneFilter::getResult()
{
    return temp2;
}


gumh::Road::Road()
{
    context=new RoadContext();
}

void gumh::Road::FindRoadPaintMarkings(Mat &pict)
{
    cout<<"======================seq : "<<context->cur_seq<<"==========="<<endl;



    if(context->cur_seq>=22){
        cout<<"debug"<<endl;
    }

    cur_input_pict=pict;
    cur_draw_pict=this->cur_input_pict.clone();


    //初步过滤
    if(lane_filter->context==nullptr){
        lane_filter->context=this->context;
    }
    Mat mid_step_result=lane_filter->nextFrame(cur_input_pict);

    //    DrawAllRectangle(pict);



    //去掉非法的区域，如人，车上的线，需结合其他来看
    //RemoveInvalidArea();

    //组织旋转矩形的拓扑关系
    BuildRelations(mid_step_result);

    FindArrow();

    //筛选符合合理车道线关系的车道线
    //    GetGoodLanes();

    //重新确认检查所选择的车道线是否符合条件
    RecheckLanes();


    //对于明显是车道线，但是没有被选中的进行进一步的分析，可以作为特殊存在的处理
    //如人字形分叉路口的另一个出口等
    //HandleSpecialOnes();


    //分析历史轨迹的情况,对没有得到支持的历史轨迹，进行判断处理
    //结合是否切道等情况，尤其对处于边缘的线进行判断，是否已经不存在了
    //UpdateHistory();


    //判断场景：转弯，路口 etc.
    //    cv::imshow("final",cur_input_pict);



    DrawAllRectangle(cur_draw_pict,cv::Scalar(0,255,255),3);


    //---draw-----//
    for(RoadPaintMarking* one:this->lane_markings){
        if(one->road_paint_type==PaintType_Arrow){
            DrawOneRectangle(cur_draw_pict,one->support_history_segs.back(),cv::Scalar(0,0,255),1);
            continue;
        }
        if(one->is_oneof_double_lane){
            one->support_history_segs.back().bounding_box_line_model.Draw(cur_draw_pict,one->draw_color,2);
        }else{
            one->support_history_segs.back().bounding_box_line_model.Draw(cur_draw_pict,one->draw_color,4);
        }
    }
    //    for(RoadPaintMarkingSeg* one:groups[0].lane_marks){
    //        one->bounding_box_line_model.Draw(getlanemat,cv::Scalar(0,255,0));
    //    }

    char text[64];
    sprintf(text,"#%ld",context->cur_seq);
    cv::Point pt(100,100);
    cv::putText(cur_draw_pict,text,pt,cv::FONT_HERSHEY_SIMPLEX,3,cv::Scalar(0,0,255));
    cv::circle(cur_draw_pict,context->varnish_pt,8,cv::Scalar(255,0,0),2);
    cv::imshow("getlanemat",cur_draw_pict);


    this->context->cur_seq++;

}


void Road::GetCandidateLaneSegGroup(vector<CandidateGroup>& groups,int top){
    groups.clear();
    if(context->segs.size()==0){
        return;
    }

    //----just return all-----//
//    CandidateGroup group;
//    for(LaneMarkingSeg& one:context->segs){
//        group.lane_marks.push_back(&one);
//    }
//    groups.push_back(group);
//    return ;


    map<long,long> next;
    bool exist=true;
    //找若干组
    for(int i=0;i<(context->segs.size()-1);i++){
        LaneMarkingSeg& one=context->segs[i];
        exist=false;
        CandidateGroup group;
        group.lane_marks.push_back(&one);

        LaneMarkingSeg* cur=&one;
        for(int j=i+1;j<context->segs.size();j++){
            LaneMarkingSeg& two=context->segs[j];
            float x_delta=two.x_btm-cur->x_btm;
            if(x_delta>600 && x_delta<900){
                //是否已存在这种关系
                if(cur==&one){//第一个
                    for(auto it=next.begin();it!=next.end();it++){
                        if(it->first==cur->id && it->second==two.id){
                            exist=true;
                            break;
                        }
                    }
                }
                if(!exist){
                    LOG(INFO)<<cur->x_btm<<" -> " << two.x_btm;
                    next[cur->id]=two.id;
                    group.lane_marks.push_back(&two);
                    cur=&two;
                }
            }else if(x_delta>900){
                //相隔太远的不用继续找了
                break;
            }

            if(exist==true){
                //这个片段在之前的片段过程中已经建立过关系了,所以不用继续了
                break;
            }

        }

        if(exist==false){
            groups.push_back(group);//正式加入
            if(group.lane_marks.size()>1){
                for(LaneMarkingSeg* each:group.lane_marks){
                    each->group_id=group.id;
                    for(LaneMarkingSeg* each2:group.lane_marks){
                        if(each!=each2){
                            //每个seg建立同组关系
                            each->same_group_segs.push_back(each2);
                        }
                    }
                }
            }
        }
    }

    //计算与历史轨迹的偏差
    for(CandidateGroup &g:groups){
        CalculateGroupLoss(g);
    }

    if(groups.size()==0){
        return;
    }
    std::sort(groups.begin(),groups.end(),candidate_segs_loss_less);

    //---输出各group---//
    int idx=0;
    for(CandidateGroup &g:groups){
        cout<<"group #"<<idx++<<",loss="<<g.loss<<" x_btm:";
        for(LaneMarkingSeg* s:g.lane_marks){
            cout<<s->x_btm<<" ";
        }
        cout<<endl;
    }
    cout<<endl;

    if(top==1){
        //找最多的一组
        int max_segs_cnt=groups[0].lane_marks.size();

        vector<LaneMarkingSeg> ss;
        std::copy(context->segs.begin(),context->segs.end(),std::back_inserter(ss));
        std::sort(ss.begin(),ss.end(),lanes_length_cmp_greator);

        //赋予group0
        CandidateGroup &group=groups[0];
        cout<<"group0 :(id="<<group.id<<")";
        for(LaneMarkingSeg* s:group.lane_marks){
            cout<<s->x_btm<<" ";
        }
        cout<<endl;

        if(max_segs_cnt==1){
            //找最长的线
            for(LaneMarkingSeg& one:context->segs){
                if(one.dir_len>context->pict_size.height/3
                        && one.group_id!=group.id){
                    cout<<"group0 加入:"<<one.x_btm<<endl;
                    one.group_id=group.id;
                    group.lane_marks.push_back(&one);
                }
            }
        }
        //看有没有长线没有被选中
        for(LaneMarkingSeg& one:context->segs){
            if(one.dir_len>context->pict_size.height/3){
                //很长的线不在这个组，要么是单独一组，要么是在其他组
                //                if(one.group_id!=group.id){//不在这个组
                //                    cout<<"group0 加入:"<<one.x_btm<<" one.group_id="
                //                       <<one.group_id<<",group.id="<<group.id<<endl;
                //                    group.lane_marks.push_back(&one);
                //                }
                int k=0;
                for(k=0;k<group.lane_marks.size();k++){
                    LaneMarkingSeg* seg=group.lane_marks[k];
                    if(seg->id==one.id){
                        break;
                    }
                }
                //                LOG(INFO)<<"one.id="<<one.id<<",one.x_btm="<<
                //                      one.x_btm<<",k="<<k<<",group.lane_marks.size()="<<group.lane_marks.size()<<endl;

                if(k==group.lane_marks.size()){
                    group.lane_marks.push_back(&one);
                    cout<<"加入long lane:"<<one.x_btm<<endl;
                    //与该long lane同组的其他也加入
                    for(LaneMarkingSeg* each2:one.same_group_segs){
                        if(!group.IsExist(each2)){
                            group.lane_marks.push_back(each2);
                        }
                    }

                }
            }
        }

        groups.resize(1);
    }
}

float Road::CalculateGroupLoss(CandidateGroup &group)
{
    if(group.lane_marks.size()==0){
        return 9999999.0;
    }
    if(this->lane_markings.size()==0){
        return 1;
    }
    float loss=0;
    int cnt=0;
    for(LaneMarkingSeg* s:group.lane_marks){
        //找到最近的历史轨迹
        float min=99999,every=0;
        for(RoadPaintMarking* lm:this->lane_markings){
            every=fabs(lm->support_history_segs.back().x_btm-s->x_btm);
            if(every<200){//group里的某些seg可能是新出现的，不在历史轨迹中的
                if(min>every){
                    min=every;
                }
            }
        }
        if(min<99999){
            cnt++;
            loss+=min;
        }
    }

    if(cnt>0){
        group.loss=loss/cnt;
    }else{
        //都没有找到合适的历史轨迹做判断
        //
        group.loss= (-1)*group.lane_marks.size();
    }
    return group.loss;
}


void Road::InitialLanes()
{

    if(context->segs.size()==0){
        //否则下面的for(int i=0;i<(context->segs.size()-1);i++){
        //会溢出变成18446744073709551615
        return;
    }

    vector<CandidateGroup> groups;
    GetCandidateLaneSegGroup(groups,1);


    if(groups.size()==0){
        return;
    }
    for(CandidateGroup &one:groups){
        one.Print();
    }

    //--------------------精选---------------------------//


    //构建车道线
    vector<LaneMarkingSeg> mid_segs;
    for(LaneMarkingSeg* each:groups[0].lane_marks){
        mid_segs.push_back(*each);
    }
    //按照x进行左到右（小到大）排序
    std::sort(mid_segs.begin(),mid_segs.end(),lanes_x_btm_cmp_less);

    RoadPaintMarking* pre_lm=nullptr;
    for(LaneMarkingSeg& each:mid_segs){

        RoadPaintMarking *lm=new RoadPaintMarking();
        lm->first_frame_seq_support=context->cur_seq;
        lm->last_frame_seq_support=context->cur_seq;
        lm->support_history_segs.push_back(each);
        lm->right_lane=nullptr;
        lm->left_lane=nullptr;

        if(pre_lm!=nullptr){
            pre_lm->right_lane=lm;
            lm->left_lane=pre_lm;

            if(lm->support_history_segs.back().x_btm -
                    pre_lm->support_history_segs.back().x_btm
                    <=100){
                //双线
                pre_lm->is_oneof_double_lane=true;
                lm->is_oneof_double_lane=true;
            }
        }
        pre_lm=lm;
        this->lane_markings.push_back(lm);
    }


}

/**
 * @brief Road::UpdateLanes
 * 有初始化的情况下
 */
void Road::UpdateLanes()
{
    //找候选的组合
    vector<CandidateGroup> groups;
//    GetCandidateLaneSegGroup(groups,1);
    //use all segs
    CandidateGroup group;
    for(LaneMarkingSeg &one:context->segs){
        group.lane_marks.push_back(&one);
    }
    groups.push_back(group);


    //---//
    for(RoadPaintMarking* lm:this->lane_markings){
        lm->ClearCandidateSegs();
    }
    vector<RoadPaintMarking*> maybe_new_lanemarks;//可能是新增的lane
    if(groups.size()>0){
        CandidateGroup &group=groups[0];
        bool fit=false;
        int newer_cnt=0;//新检测的线
        for(LaneMarkingSeg* each:group.lane_marks){
            cout<<"group seg ,x_btm="<<each->x_btm<<endl;
            fit=false;
            for(RoadPaintMarking* lm:this->lane_markings){
                cout<<"   history lane x_btm="<<lm->support_history_segs.back().x_btm<<endl;
                if(fabs(each->x_btm -
                        lm->support_history_segs.back().x_btm)
                        <100
                        &&
                        fabs(each->blob_angle_deg - lm->support_history_segs.back()
                             .blob_angle_deg)<10){
                    lm->AddCandidateSupportSeg(*each);

//                    lm->support_history_segs.push_back(*each);
//                    lm->last_frame_seq_support=context->cur_seq;
                    fit=true;
                    cout<<each->x_btm<< " is a candidate of :"<<lm->support_history_segs.back().x_btm<<endl;
                    break;
                }
            }
            //对于新发现的，不在任何一个历史轨迹中的进行处理 TODO
            if(fit==false){
                cout<<"新增一个候选RoadPaintMarking.x_btm="<<each->x_btm<<endl;
                ++newer_cnt;
                RoadPaintMarking *lm=new RoadPaintMarking();
                lm->first_frame_seq_support=context->cur_seq;
                lm->last_frame_seq_support=context->cur_seq;
                lm->support_history_segs.push_back(*each);
                lm->right_lane=nullptr;
                lm->left_lane=nullptr;

                //加入可能是新增的队列
                maybe_new_lanemarks.push_back(lm);
            }
        }
    }


    //对于已有lane加入各个候选者，进行全局优化
    ChooseBestCandidateSegs();

    //处理可能是新增的，对其进行合理性判断
    HandleMaybeNewerLanes(maybe_new_lanemarks);

    //    cout<<"this->lane_markings.size="<<this->lane_markings.size()<<endl;
    //对历史轨迹进行处理
    vector<RoadPaintMarking*> keeps,deletes;
    for(RoadPaintMarking* lm:this->lane_markings){
        //        cout<<"lane mark history x_btm:";
        //        for(RoadPaintMarkingSeg &s:lm->support_history_segs){
        //            cout<<s.x_btm<<" ";
        //        }
        //        cout<<endl;
        //简单清理，先验证主体
        if(context->cur_seq-lm->last_frame_seq_support<=1){
            keeps.push_back(lm);
            //每100帧清理一次，保留50
            if(lm->support_history_segs.size()>100){
                vector<LaneMarkingSeg> tmp;
                int size=lm->support_history_segs.size();
                for(int k=size-50;k<size;k++){
                    tmp.push_back(lm->support_history_segs[k]);
                }
                cout<<"remain seg size= "<<tmp.size()<<endl;
                lm->support_history_segs.clear();
                for(int k=0;k<tmp.size();k++){
                    lm->support_history_segs.push_back(tmp[k]);
                }
            }
        }else{
            deletes.push_back(lm);
        }
    }
    if(deletes.size()>0){
        this->lane_markings.clear();
        std::copy(keeps.begin(),keeps.end(),back_inserter(this->lane_markings));
        for(RoadPaintMarking* lm:deletes){
            delete lm;
        }
    }

    //总体排序
    std::sort(this->lane_markings.begin(),this->lane_markings.end(),lanes_x_btm_cmp_less_ptr);

}

void Road::HandleMaybeNewerLanes(vector<RoadPaintMarking *> &maybe_newer)
{
    if(maybe_newer.size()==0){
        return ;
    }
    cout<<"处理新增的候选车道线"<<endl;
    vector<RoadPaintMarking*> goods;
    bool get=false;
    float x_delta=-0,x_delta2=0;

    cout<<"已有的lane marking的x信息："<<endl;
    for(RoadPaintMarking* st:this->lane_markings){
        cout<<"lane x_btm="<<st->support_history_segs.back().x_btm<<endl;
    }

    for(RoadPaintMarking* each:maybe_newer){
        get = false;
        //与最近的两条已有线的距离
        RoadPaintMarking* pre=nullptr;
        RoadPaintMarking* stop=nullptr;
        RoadPaintMarking* cur=nullptr;
        for(RoadPaintMarking* st:this->lane_markings){
            cur=st;
            if(each->support_history_segs.back().x_btm < st->support_history_segs.back().x_btm){
                stop=st;
                break;
            }
            pre=st;
        }
        if(stop==nullptr){
            //新出现的可疑段出现在最右侧
            x_delta=      each->support_history_segs.back().x_btm - cur->support_history_segs.back().x_btm;
            //距离合适或者
            //如果待选的是长线，直接纳入
            if( IsAProperDistanceToNearby(each,cur)
                    || each->support_history_segs.back().IsReallyLengthSeg()){
                //加入选择，先建立关系
                cur->right_lane=each;
                each->left_lane=cur;
                //先不加入队列
                get=true;
            }
        }else{
            //介于某两者之间,stop在其右侧
            x_delta=stop->support_history_segs.back().x_btm - each->support_history_segs.back().x_btm;
            //与右侧的距离合适
            if(IsAProperDistanceToNearby(each,stop)
                    || each->support_history_segs.back().IsReallyLengthSeg()){
                if(pre!=nullptr){
                    //比较与左侧的距离
                    x_delta2=each->support_history_segs.back().x_btm - pre->support_history_segs.back().x_btm;
                    if(IsAProperDistanceToNearby(each,pre)
                            ||
                            each->support_history_segs.back().IsReallyLengthSeg()){
                        pre->right_lane=each;
                        stop->left_lane=each;
                        each->left_lane=pre;
                        each->right_lane=stop;

                        get=true;
                    }
                }else{
                    //最左侧
                    stop->left_lane=each;
                    each->right_lane=stop;

                    get=true;

                }
            }
        }
        if(get){
            goods.push_back(each);
            cout<<"确认增加新增的候选lane:"<<each->support_history_segs.back().x_btm;
            if(pre!=nullptr){
                cout<<", 左边线x_btm="<<pre->support_history_segs.back().x_btm;
            }else{
                cout<<",无左边线";
            }
            if(stop!=nullptr){
                cout<<", 右边线x_btm="<<stop->support_history_segs.back().x_btm;
            }else{
                cout<<",无右边线";
            }
            cout<<endl;

        }else{
            cout<<"排除新增的候选lane:"<<each->support_history_segs.back().x_btm;
            if(pre!=nullptr){
                cout<<", 左边线x_btm="<<pre->support_history_segs.back().x_btm;
            }else{
                cout<<",无左边线";
            }
            if(stop!=nullptr){
                cout<<", 右边线x_btm="<<stop->support_history_segs.back().x_btm;
            }else{
                cout<<",无右边线";
            }
            cout<<endl;
        }
    }

    //对于被排除的长线，需要进行进一步考察


    if(goods.size()>0){
        //增加goods
        std::copy(goods.begin(),goods.end(),back_inserter(this->lane_markings));
        //总体排序
        std::sort(this->lane_markings.begin(),this->lane_markings.end(),lanes_x_btm_cmp_less_ptr);

    }

}


void CalculateVarianceValue(std::vector<RoadPaintMarking*> all,int cur_idx,vector<LaneMarkingSeg*>& selecting_segs,vector<LaneMarkingSeg*>& selected_segs,VarianceValue& value){
    if(cur_idx>=all.size()){
        return;
    }
    if(all.size()==0){
        return;
    }
    RoadPaintMarking* cur=all[cur_idx];
    if(cur_idx==all.size()-1){
        for(LaneMarkingSeg* each:cur->candidate_segs){
            //get the least value
            selecting_segs.push_back(each);
            float delta_x=0;
            vector<float> xs;
            for(int i=0;i<selecting_segs.size();i++){
                LaneMarkingSeg* one=selecting_segs[i];
                RoadPaintMarking *lm=all[i];
                xs.push_back(/*fabs(*/one->x_btm-lm->support_history_segs.back().x_btm);
                delta_x+=xs.back();
            }
            float mean_x=delta_x/selecting_segs.size();
            float variance_x=0;
            for(int i=0;i<selecting_segs.size();i++){
                variance_x+=pow(xs[i]-mean_x,2);
            }
            variance_x/=(selecting_segs.size());
            variance_x=sqrt(variance_x);
            if(variance_x<value.variance_delta_x){
                value.variance_delta_x=variance_x;
                value.mean_delta_x=mean_x;
                selected_segs.clear();
                std::copy(selecting_segs.begin(),selecting_segs.end(),std::back_inserter(selected_segs));
            }
            selecting_segs.resize(selecting_segs.size()-1);
        }
        return;
    }else{
       for(LaneMarkingSeg* each:cur->candidate_segs){
           selecting_segs.push_back(each);
           CalculateVarianceValue(all,cur_idx+1,selecting_segs,selected_segs,value);
           selecting_segs.resize(selecting_segs.size()-1);
       }
    }
}

void Road::ChooseBestCandidateSegs()
{
    //use L2
    //sqrt((slope-slope_)^2+delta_x^2)

    //
    std::vector<RoadPaintMarking*> all;
    for(RoadPaintMarking* one:this->lane_markings){
        if(one->candidate_segs.size()>0){
            all.push_back(one);
        }
    }
    if(all.size()>0){
        vector<LaneMarkingSeg*> selecting_segs,selected_segs;
        VarianceValue value;
        CalculateVarianceValue(all,0,selecting_segs,selected_segs,value);
        for(int i=0;i<all.size();i++){
            all[i]->AddSupportSeg(*selected_segs[i],context->cur_seq);
        }
        cout<<"choose segs,mean_delta_x="<<value.mean_delta_x
           <<",variance_delta_x="<<value.variance_delta_x
          <<endl;
    }


}

bool Road::IsAProperDistanceToNearby(RoadPaintMarking *candidate, RoadPaintMarking *exists_one)
{
    float xdelta=candidate->support_history_segs.back().x_btm-
            exists_one->support_history_segs.back().x_btm;
    if(candidate->support_history_segs.back().x_btm < exists_one->support_history_segs.back().x_btm){
        xdelta=exists_one->support_history_segs.back().x_btm-
                candidate->support_history_segs.back().x_btm;
    }

    if(xdelta>600){
        return true;
    }
    return false;

}

void gumh::Road::BuildRelations(Mat &mid_step_result)
{

    //去掉明显不对的车道线
    RemoveBadPositionLines();

    //    LOG(INFO)<<"合并前，segs.size="<<context->segs.size()<<endl;
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(mid_step_result, contours,
                 hierarchy, CV_RETR_CCOMP,
                 CV_CHAIN_APPROX_SIMPLE);

    if (!contours.empty())
    {
        for (size_t i=0; i<contours.size(); ++i)
        {
            drawContours(cur_draw_pict,contours,i, Scalar(255), CV_FILLED, 8);
            //旋转矩形
        }
    }


    //对于所有的旋转矩形，进行合并处理
    int comb_num=0;
    vector<std::pair<long,long>> comb_pair;
    vector<std::pair<long,long>> no_comb;

    bool skip=false;
    do{
        comb_pair.clear();
        no_comb.clear();
        comb_num=0;

        for(LaneMarkingSeg &one:context->segs){
            for(LaneMarkingSeg &another:context->segs){
                skip=false;
                //不在已经匹配的当中
                for(pair<long,long> cp:comb_pair){
                    if(cp.first == one.id && cp.second == another.id
                            ||
                            cp.first == another.id && cp.second == one.id){
                        skip=true;
                        break;
                    }
                }
                if(skip==true) continue;
                for(pair<long,long> cp:no_comb){
                    if(cp.first == one.id && cp.second == another.id
                            ||
                            cp.first == another.id && cp.second == one.id){
                        skip=true;
                        break;
                    }
                }
                if(skip==true) continue;

                pair<long,long> cp(one.id,another.id);
                if(one.CanCombine(another)){
                    //                    cout<<"可以合并seg，角度:"<<one.blob_angle_deg<<" 和 "<<another.blob_angle_deg<<endl;
                    ++comb_num;
                    one.wait_to_combine_segs.push_back(&another);
                    comb_pair.push_back(cp);
                }else{
                    no_comb.push_back(cp);
                }
            }
        }

        //do the real combine job
        if(comb_num>0){
            CombineSegs();
        }
    }while(comb_num>0);

    //    LOG(INFO)<<"合并后，segs.size="<<context->segs.size()<<endl;

    //organize the lanes according to the x-value in the bottom of the picture
    std::sort(context->segs.begin(),context->segs.end(),lanes_x_btm_cmp_less);

    cout<<"排序后与图片底下边的交点x值：";
    //    cv::Mat tmp_pict=cur_input_pict.clone();
    //    imshow("tmp-pict",tmp_pict);
    //    cv::waitKey(0);
    for(LaneMarkingSeg& a:context->segs){
        cout<<a.x_btm<<" ";
        //        DrawOneRectangle(tmp_pict,a,cv::Scalar(0,0,255));
        //        imshow("tmp-pict",tmp_pict);
        //        cv::waitKey(0);
    }
    cout<<endl;


    if(lane_markings.size()==0){
        InitialLanes();
    }else{
        UpdateLanes();
    }


}

bool between(float x,float min,float max){
    if(x>min && x<max){
        return true;
    }else
        return false;
}

void Road::FindArrow()
{
    return;
    //基于长线中间的短线
    //两条长线，距离处于一个车道的合适范围内，
    //两者中间有一条线(非长线），
    //两条长线的累计可信度是中间线的两倍
    //context->segs拍好了序
    if(context->segs.size()==0){
        return;
    }
    for(int i=0;i<this->lane_markings.size()-3;i++){
        RoadPaintMarking* rpm=this->lane_markings[i];
        RoadPaintMarking* rpmr=this->lane_markings[i+1];
        RoadPaintMarking* rpmrr=this->lane_markings[i+2];

        if(rpm->support_history_segs.back().IsReallyLengthSeg()
                && !rpmr->support_history_segs.back().IsReallyLengthSeg()
                && rpmrr->support_history_segs.back()
                .IsReallyLengthSeg()

                &&
               ( between(rpmr->support_history_segs.back().x_btm-
                 rpm->support_history_segs.back().x_btm,200,400)
                 ||between(rpmrr->support_history_segs.back().x_btm-
                          rpmr->support_history_segs.back().x_btm
                 ,200,400)
                )){
            rpm->right_lane->road_paint_type=PaintType_Arrow;
        }
    }

}

void Road::RecheckLanes()
{

}


void Road::AnalyzeByVarnishPoint(){
    if(context->segs.size()==0){
        return;
    }

    if(context->segs.size()==1){
        LaneMarkingSeg & one=context->segs[0];
        if(one.dir_len> context->pict_size.height/3){
            return;
        }
        if(context->varnish_pt.x!=-1 ||
                context->varnish_pt.y!=-1){
            if(one.bounding_box_line_model.GetDistanceToPoint(context->varnish_pt)
                    >20){
                context->segs.clear();
            }
        }
        return;
    }

    //    if(context->segs.size()<3){
    //        //起码有三条线
    //        if(context->varnish_pt.x==-1 && context->varnish_pt.y==-1){
    //            return;
    //        }else{
    //            //计算到灭点的距离
    //            for(RoadPaintMarkingSeg &seg:context->segs){
    //                if(seg.dir_len<context->pict_size.height/3 &&
    //                   seg.bounding_box_line_model.GetDistanceToPoint(context->varnish_pt) > context->allow_max_dist_from_varnish_pt){

    //                }
    //            }
    //        }
    //    }
    //    LOG(INFO)<<"灭点分析前，剩余seg数量:"<<context->segs.size();
    //灭点
    //求两两的交点


    vector<SegInterceptPoint> good_pt_pairs;
    vector<SegInterceptPoint> bad_pt_pairs;
    vector<LaneMarkingSeg*> in_good_pt_segs,in_bad_pt_segs;
    vector<LofPoint> lps;
    float x_total=0,y_total=0,good_p_cnt=0;
    for(int i=0;i<context->segs.size()-1;i++){
        LaneMarkingSeg &seg=context->segs[i];
        for(int j=i+1;j<context->segs.size();j++){
            LaneMarkingSeg &seg2=context->segs[j];
            cv::Point2f pt;
            int ret=seg.bounding_box_line_model.GetInterceptPoint(seg2.bounding_box_line_model,pt);
            if(ret==2){
                seg.pt_with_others.push_back(pt);
                seg2.pt_with_others.push_back(pt);

                LofPoint lp;
                lp.x=pt.x;
                lp.y=pt.y;
                lps.push_back(lp);



                SegInterceptPoint sip;
                sip.pt=pt;
                sip.one=&seg;
                sip.two=&seg2;
//                cout<<"context->varnish_p_y_value="<<context->varnish_p_y_value
//                   <<",context->allow_max_y_offset="<<context->allow_max_y_offset
//                  <<",pt.y="<<pt.y<<endl;
                if(pt.y>=context->varnish_p_y_value-context->allow_max_y_offset
                        && pt.y<=context->varnish_p_y_value+context->allow_max_y_offset){
                    sip.is_bad=false;
                    good_pt_pairs.push_back(sip);

                    x_total+=pt.x;
                    y_total+=pt.y;
                    ++good_p_cnt;

                    in_good_pt_segs.push_back(&seg);
                    in_good_pt_segs.push_back(&seg2);
                    //                    cout<<"good pt="<<pt<<",seg1 dir_len="<<seg.dir_len
                    //                       <<",angle="<<seg.blob_angle_deg<<
                    //                         ",seg2 dir_len="<<seg2.dir_len
                    //                      <<",angle="<<seg2.blob_angle_deg<<endl;
                }else{
                    sip.is_bad=true;
                    bad_pt_pairs.push_back(sip);

                    in_bad_pt_segs.push_back(&seg);
                    in_bad_pt_segs.push_back(&seg2);
                    //                    cout<<"bad pt="<<pt<<",seg1 dir_len="<<seg.dir_len
                    //                       <<",angle="<<seg.blob_angle_deg<<
                    //                         ",seg2 dir_len="<<seg2.dir_len
                    //                      <<",angle="<<seg2.blob_angle_deg<<endl;
                }
            }
        }
    }

    //主灭点更新
    if(in_good_pt_segs.size()>0){
        if(context->varnish_pt.x==-1 && context->varnish_pt.y==-1){
            context->varnish_pt.x=x_total/good_p_cnt;
            context->varnish_pt.y=y_total/good_p_cnt;
        }else{
            context->varnish_pt.x+=x_total/good_p_cnt;
            context->varnish_pt.y+=y_total/good_p_cnt;
            context->varnish_pt.x/=2;
            context->varnish_pt.y/=2;
        }
    }

    //统计good point的线到灭点的平均距离
    float avg_dist_to_varnish=context->allow_max_dist_from_varnish_pt
            ,dist_total=0;
    if(in_good_pt_segs.size()>0){
        for(LaneMarkingSeg* seg:in_good_pt_segs){
            dist_total+=seg->bounding_box_line_model.GetDistanceToPoint(context->varnish_pt);
        }
        avg_dist_to_varnish=dist_total/in_good_pt_segs.size();
    }

    //    cout<<"varnish point="<<context->varnish_pt<<endl;
    //    cout<<"in_bad_pt_segs.size="<<in_bad_pt_segs.size()<<endl;
    //对于处在bad point segs队列中的每一项，都进行分析
    vector<LaneMarkingSeg*> for_remove_bad_segs;
    vector<LaneMarkingSeg*> for_remove_bad_segs_but_long_line;//被灭点排除掉的，但属于长线
    if(in_bad_pt_segs.size()>0){
        vector<LaneMarkingSeg*> handled_segs;
        bool handled=false;
        for(LaneMarkingSeg* each:in_bad_pt_segs){
            handled=false;
            int cnt=0;
            for(LaneMarkingSeg* two:in_good_pt_segs){
                if(each==two){
                    ++cnt;
                }
            }
            each->good_pt_cnt=cnt;

            //            cout<<"分析badpt队列的组成元素seg:dir-len="<<each->dir_len
            //               <<",blob_angle_deg="
            //              <<each->blob_angle_deg
            //             <<", 涉及了good pt数量："<<cnt<<endl;


            for(LaneMarkingSeg* a:handled_segs){
                if(a->id==each->id){
                    handled=true;
                }
            }
            if(!handled){
                handled_segs.push_back(each);
            }
        }

        std::sort(handled_segs.begin(),handled_segs.end(),cmp_good_pt_less);
        //可以用循环的方式去除一个怀疑对象后，看剩余的坏点的情况
        for(LaneMarkingSeg* a:handled_segs){
            //            cout<<"分析怀疑seg,good_pt_cnt="<<a->good_pt_cnt<<",dir_len="<<a->dir_len<<",angle="<<a->blob_angle_deg<<endl;

            //如果把这个seg去除，坏点的数量是否消失
            if( a->dir_len < context->pict_size.height/3){
                if(a->good_pt_cnt<3 ){
                    for_remove_bad_segs.push_back(a);
                }else if(context->varnish_pt.x!=-1 ||
                         context->varnish_pt.y!=-1){
                    //比较与灭点的平均距离
                    if(a->bounding_box_line_model.GetDistanceToPoint(context->varnish_pt)
                            > avg_dist_to_varnish*1.5){
                        //                        cout<<"通过与灭点距离去除seg: dir_len="<<a->dir_len<<
                        //                              ",blob_angle_deg="<<a->blob_angle_deg<<endl;
                        for_remove_bad_segs.push_back(a);
                    }
                }
            }
        }
    }

    if(for_remove_bad_segs.size()>0){
        bool is_bad=false;
        vector<LaneMarkingSeg> goods;
        std::copy(context->segs.begin(),context->segs.end(),back_inserter(goods));
        context->segs.clear();
        for(LaneMarkingSeg seg:goods){
            is_bad=false;
            for(LaneMarkingSeg* each:for_remove_bad_segs){
                if(seg.id==each->id){
                    //                    cout<<"通过灭点去除bad seg: dir_len="<<each->dir_len<<","<<each->blob_angle_deg<<endl;
                    is_bad=true;
                    break;
                }
            }
            if(!is_bad){
                context->segs.push_back(seg);
            }
        }
    }

    //重新计算灭点
    if(context->segs.size()==0){
        return;
    }
    good_pt_pairs.clear();
    in_good_pt_segs.clear();
    x_total=0,y_total=0,good_p_cnt=0;
    for(int i=0;i<context->segs.size()-1;i++){
        LaneMarkingSeg &seg=context->segs[i];
        for(int j=i+1;j<context->segs.size();j++){
            LaneMarkingSeg &seg2=context->segs[j];
            cv::Point2f pt;
            int ret=seg.bounding_box_line_model.GetInterceptPoint(seg2.bounding_box_line_model,pt);
            if(ret==2){
                SegInterceptPoint sip;
                sip.pt=pt;
                sip.one=&seg;
                sip.two=&seg2;
                if(pt.y>=context->varnish_p_y_value-context->allow_max_y_offset
                        && pt.y<=context->varnish_p_y_value+context->allow_max_y_offset){
                    sip.is_bad=false;
                    good_pt_pairs.push_back(sip);

                    x_total+=pt.x;
                    y_total+=pt.y;
                    ++good_p_cnt;

                    in_good_pt_segs.push_back(&seg);
                    in_good_pt_segs.push_back(&seg2);
                    //                    cout<<"good pt="<<pt<<",seg1 dir_len="<<seg.dir_len
                    //                       <<",angle="<<seg.blob_angle_deg<<
                    //                         ",seg2 dir_len="<<seg2.dir_len
                    //                      <<",angle="<<seg2.blob_angle_deg<<endl;
                }
            }
        }
    }

    //主灭点更新
    if(in_good_pt_segs.size()>0){
        if(context->varnish_pt.x==-1 && context->varnish_pt.y==-1){
            context->varnish_pt.x=x_total/good_p_cnt;
            context->varnish_pt.y=y_total/good_p_cnt;
        }else{
            context->varnish_pt.x+=x_total/good_p_cnt;
            context->varnish_pt.y+=y_total/good_p_cnt;
            context->varnish_pt.x/=2;
            context->varnish_pt.y/=2;
        }
    }



//    //    LOG(INFO)<<"灭点分析后，剩余seg数量:"<<context->segs.size();

}

void Road::RemoveBadPositionLines()
{


    //-----灭点分析-----//
    AnalyzeByVarnishPoint();

    //角度与位置
    //左边的片段，角度是负数的,如果是正数，就可能有问题
    //为了更可靠，如果低点（y大）很靠下，则就可以去除

}
bool LaneMarkingSeg::CanCombine(LaneMarkingSeg &another)
{
    if(this->id == another.id){
        return false;
    }
    if(this->blob_angle_deg>=0
            &&
            another.blob_angle_deg>=0
            || this->blob_angle_deg<0 && another.blob_angle_deg<0){

        if(fabs(this->blob_angle_deg-another.blob_angle_deg)<5){
            //角度ok了，看前后关系
            //穿越关系
            if(IsMidLineGoThroughAnother(another)
                    || another.IsMidLineGoThroughAnother(*this)){

                //                this->Print();
                //                another.Print();
                bool ret=true;
                return ret;
            }
        }
    }

    return false;
}

void LaneMarkingSeg::Combine(LaneMarkingSeg &another)
{

    another.is_combined=true;
    this->all_pts.push_back(another.contour_pts);
    vector<cv::Point> allpts;
    for(vector<cv::Point> ap:all_pts){
        std::copy(ap.begin(),ap.end(),std::back_inserter(allpts));
    }

    //combine的具体策略之一:直接用点来产生大框
    this->ParseValue(allpts);

    this->is_a_combine_segs=true;

}

bool LaneMarkingSeg::isContainPt(Point2f &pt)
{
    std::vector<cv::Point2f> inps;
    for(int l=0;l<4;l++)
        inps.push_back(rect_points[l]);
    if(cv::pointPolygonTest(inps,pt,false)==1){
        return true;
    }else{
        return false;
    }
}

bool LaneMarkingSeg::IsReallyLengthSeg()
{
    return dir_len> context->pict_size.height/3;
}

void LaneMarkingSeg::Print()
{
    std::cout<<">>>>>>>>>>>>>RoadPaintMarkingSeg#"<<this->id<<" Print begin>>>>>>>>>>>>"<<endl;
    std::cout<<"x_btm="<<this->x_btm<<",blob_angle_deg="<<blob_angle_deg<<endl;

    std::cout<<"rect_points:";
    for(int i=0;i<4;i++){
        std::cout<<rect_points[i]<<" ";
    }
    std::cout<<endl<<"mid_line:";
    bounding_box_line_model.Print();
    std::cout<<endl<<"edge lines:";
    for(int i=0;i<4;i++){
        edge_lines[i].Print();
    }

    std::cout<<"<<<<<<<<<<<<<RoadPaintMarkingSeg#"<<this->id<<" Print  end  <<<<<<<<<<<<"<<endl;

}
void CombineOneSeg(LaneMarkingSeg& one){
    if(one.wait_to_combine_segs.size()==0){
        return;
    }
    for(LaneMarkingSeg* ch:one.wait_to_combine_segs){
        CombineOneSeg(*ch);
    }
    one.is_a_combine_segs=true;
    for(LaneMarkingSeg* ch:one.wait_to_combine_segs){
        //real combine
        one.Combine(*ch);
        cout<<"合并segs,角度为 :"<<one.blob_angle_deg<<" 和 "<<ch->blob_angle_deg<<endl;
    }
    one.wait_to_combine_segs.clear();
}

void Road::CombineSegs()
{
    for(LaneMarkingSeg& one:context->segs){
        if(one.wait_to_combine_segs.size()>0)
            CombineOneSeg(one);
    }
    vector<LaneMarkingSeg> tmp;
    for(LaneMarkingSeg one:context->segs){
        if(one.is_combined){
            continue;
        }
        tmp.push_back(one);
    }
    context->segs.clear();
    std::copy(tmp.begin(),tmp.end(),std::back_inserter(context->segs));
}

bool Road::lanes_x_btm_cmp_less_ptr(RoadPaintMarking *a, RoadPaintMarking *b)
{
    return a->support_history_segs.back().x_btm < b->support_history_segs.back().x_btm;
}

bool Road::lanes_x_btm_cmp_less(LaneMarkingSeg &a, LaneMarkingSeg &b)
{
    return a.x_btm<b.x_btm;
}

bool Road::lanes_length_cmp_greator(LaneMarkingSeg &a, LaneMarkingSeg &b)
{
    return a.dir_len>b.dir_len;
}

void Road::DrawOneRectangle(Mat& pict,LaneMarkingSeg& one,cv::Scalar col,int thickness,cv::Scalar endp_col){
    cv::Point2f rect_points[4];
    one.rotated_rect.points(rect_points);
    for( int j = 0; j < 4; j++ )
    {
        line( pict, rect_points[j], rect_points[(j+1)%4], col, thickness, 8 );
        circle(pict,rect_points[j],3,endp_col,1);
//        cout<<"endp["<<j<<"]="<<rect_points[j]<<endl;
        char text[64];
        if(j==0)
            sprintf(text,"a[%.2f]",one.blob_angle_deg);
        else if(j==2)
            sprintf(text,"l%.2fh%d",one.dir_len,context->pict_size.height);
        if(j==0 || j==2)
            cv::putText(pict,text,rect_points[j],cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,255));
    }
    //sub rect
    for(RotatedRect &each:one.sub_rects){
        each.points(rect_points);
        for( int j = 0; j < 4; j++ )
        {
            circle(pict,rect_points[j],2,cv::Scalar(0,255,0),1);
        }
    }

}

void Road::DrawAllRectangle(Mat& pict,cv::Scalar col,int thickness)
{
    //    cout<<"DrawAllRectangle ::context->segs.size="<<context->segs.size()<<endl;
    for(LaneMarkingSeg& one:context->segs){
        if(one.dir_len<context->pict_size.height/3)
            DrawOneRectangle(pict,one,col,thickness);
        else
            DrawOneRectangle(pict,one,cv::Scalar(0,255,0),thickness);
    }
}




gumh::LaneMarkingSeg::LaneMarkingSeg(RoadContext* ctx,vector<Point> &pts)
{
    id=clock();
    this->context=ctx;
    ParseValue(pts);
}

void LaneMarkingSeg::ParseValue(vector<Point> &pts)
{
    contour_pts=pts;
    all_pts.push_back(pts);

    this->rotated_rect=cv::minAreaRect(pts);
    sz              = rotated_rect.size;
    bounding_width  = sz.width;
    bounding_length = sz.height;
    rotated_rect.points(rect_points);


    //openCV selects length and width based on their orientation
    //so angle needs to be adjusted accordingly
    blob_angle_deg = rotated_rect.angle;
    if (bounding_width < bounding_length){
        blob_angle_deg = 90 + blob_angle_deg;
        dir_len=bounding_length;
        short_len=bounding_width;
    }else{
        dir_len=bounding_width;
        short_len=bounding_length;
    }
    //find the highest and lowest points
    highest_pt.x=0,highest_pt.y=1000000;
    lowest_pt.x=0,lowest_pt.y=-1;
    for(int i=0;i<4;i++){
        if(rect_points[i].y>lowest_pt.y){
            lowest_pt.x=rect_points[i].x;
            lowest_pt.y=rect_points[i].y;
        }
        if(rect_points[i].y<highest_pt.y){
            highest_pt.x=rect_points[i].x;
            highest_pt.y=rect_points[i].y;
        }
    }

    //中间线直线方程
    cv::Point2f mid_p1,mid_p2;
    if(this->blob_angle_deg<0){
        //左边
        mid_p1.x=(rect_points[0].x+rect_points[1].x)/2;
        mid_p1.y=(rect_points[0].y+rect_points[1].y)/2;

        mid_p2.x=(rect_points[2].x+rect_points[3].x)/2;
        mid_p2.y=(rect_points[2].y+rect_points[3].y)/2;

    }else{
        mid_p1.x=(rect_points[0].x+rect_points[3].x)/2;
        mid_p1.y=(rect_points[0].y+rect_points[3].y)/2;

        mid_p2.x=(rect_points[1].x+rect_points[2].x)/2;
        mid_p2.y=(rect_points[1].y+rect_points[2].y)/2;
    }
    vector<cv::Point2f> mid_pts;
    mid_pts.push_back(mid_p1);
    mid_pts.push_back(mid_p2);
    bounding_box_line_model.SetData(mid_pts);

    //与图片底边交点
    bounding_box_line_model.GetX(context->pict_size.height,x_btm);

    //四条边线方程
    for(int i=0;i<4;i++){
        vector<cv::Point2f> pts;
        pts.push_back(rect_points[i]);
        pts.push_back(rect_points[(i+1)%4]);
        edge_lines[i].SetData(pts);
    }


    //分几段拟合直线
    FitMultiLines();
}

vector<cv::RotatedRect> LaneMarkingSeg::SplitMultiSubSegs(int cnt){

    vector<cv::RotatedRect> tmpsubs;
    if(cnt<=0){
        return tmpsubs;
    }
    //计算出来的实际的每段的长度
    float each_len=dir_len/cnt;

    vector<RotatedRect> rects;
    vector<float> xl_s,yl_s,xr_s,yr_s;//分割出来的左边和右边的x坐标、y坐标值

    if(this->blob_angle_deg<0){
        //左边线
        float xl_1=this->rect_points[1].x;
        float yl_1=this->rect_points[1].y;
        float xl_2=this->rect_points[2].x;
        float yl_2=this->rect_points[2].y;

        float xr_1=this->rect_points[0].x;
        float yr_1=this->rect_points[0].y;
        float xr_2=this->rect_points[3].x;
        float yr_2=this->rect_points[3].y;

        float delta_x=fabs((xl_2-xl_1)/cnt);
        float delta_y=fabs((yl_2-yl_1)/cnt);

        for(int i=0;i<=cnt;i++){ //因为要包括末端位置，这里是小于等于
            xl_s.push_back(xl_1+delta_x*i);
            yl_s.push_back(yl_1-delta_y*i);

            xr_s.push_back(xr_1+delta_x*i);
            yr_s.push_back(yr_1-delta_y*i);
        }

        //组合旋转矩形
        for(int i=0;i<xl_s.size()-1;i++){
            //const Point2f& center, const Size2f& size, float angle
            cv::Point2f center;
            center.x=(xl_s[i]+xl_s[i+1]+xr_s[i]+xr_s[i+1])/4;
            center.y=(yl_s[i]+yl_s[i+1]+yr_s[i]+yr_s[i+1])/4;
            cv::Size2f size(each_len,short_len);
            RotatedRect rect(center,size,rotated_rect.angle);

            tmpsubs.push_back(rect);
            //找到属于这个子框的contour点
        }
    }
}

void LaneMarkingSeg::FitMultiLines()
{
    //用于拟合的每段的长度不能小于这个值
    float min_len=context->pict_size.height/6;
    //如果比短的边还要短，则要进行保护，否则拟合出来的线的朝向可能就错了
    if(min_len<=2*short_len){
        min_len=short_len*2;//不能小于短边的两倍
    }
    int cnt=dir_len/min_len;
    if(cnt==0){
        cnt=1;
    }

    vector<cv::RotatedRect> tmpsubs= SplitMultiSubSegs(cnt);
    sub_rects.clear();;
    std::copy(tmpsubs.begin(),tmpsubs.end(),std::back_inserter(sub_rects));


}

bool LaneMarkingSeg::IsMidLineGoThroughAnother(LaneMarkingSeg &another)
{

    cv::Point2f pts[4];
    int cnt=0;
    for(int i=0;i<4;i++){
        bool ok=this->bounding_box_line_model.GetInterceptPoint(another.edge_lines[i],pts[i]);
        if(ok){
            float min_x=another.rect_points[i].x;
            float max_x=another.rect_points[i].x;
            if(min_x>another.rect_points[(i+1)%4].x){
                min_x=another.rect_points[(i+1)%4].x;
            }
            if(max_x<another.rect_points[(i+1)%4].x){
                max_x=another.rect_points[(i+1)%4].x;
            }

            float min_y=another.rect_points[i].y;
            float max_y=another.rect_points[i].y;
            if(min_y>another.rect_points[(i+1)%4].y){
                min_y=another.rect_points[(i+1)%4].y;
            }
            if(max_y<another.rect_points[(i+1)%4].y){
                max_y=another.rect_points[(i+1)%4].y;
            }

            if(pts[i].x>=min_x && pts[i].x<=max_x
                    && pts[i].y>=min_y && pts[i].y<=max_y){
                ++cnt;
                if(cnt>=2){
                    return true;
                }
            }

        }
    }

    return false;


}

StraightLine::StraightLine(vector<Point2f> &pts)
{
    SetData(pts);
}

StraightLine::StraightLine()
{
    is_valid=false;
}

bool StraightLine::SetData(vector<cv::Point2f>& pts)
{
    if(pts.size()<2){
        this->is_valid=false;
    }
    cv::Vec4f line;
    cv::fitLine(pts,
                line,
                CV_DIST_HUBER   ,
                0,
                0.01,
                0.01);

    double cos_theta = line[0];
    double sin_theta = line[1];
    double x0 = line[2], y0 = line[3];

    //    LOG(INFO)<<"line[0]="<<line[0]<<",line[1]="<<line[1]
    //            <<",line[2]="<<line[2]<<",line[3]="<<line[3];

    if(sin_theta==1){
        //vertical line,x value is fixed
        this->is_vertical=true;
        //k ,b has no define
        if(line[2]>=0){
            phi=0;
            rho=line[2];
        }else{
            phi=180;
            rho=line[2];
        }
    }else{
        phi = atan2(sin_theta, cos_theta) + PI / 2.0;
        rho = y0 * cos_theta - x0 * sin_theta;

        phi=phi/PI * 180;//to degree

        //        LOG(INFO)<< "phi = " << phi ;
        //        LOG(INFO) << "rho = " << rho ;

        k = sin_theta / cos_theta;

        b = y0 - k * x0;
    }
    //    LOG(INFO)<<"k="<<k<<",b="<<b;

    //---post check---//
    this->is_valid=true;

    //    LOG(INFO)<<"phi="<<phi;
    //    LOG(INFO)<<"fabs(phi)="<<fabs(phi);
    //    LOG(INFO)<<"fabs(fabs(phi)-180)="<<fabs(fabs(phi)-180);
    //    LOG(INFO)<<"fabs(fabs(phi)-90)="<<fabs(fabs(phi)-90);
    //    LOG(INFO)<<"fabs(fabs(phi)-270)="<<fabs(fabs(phi)-270);

    if(fabs(phi)<0.01 ||
            fabs(fabs(phi)-180)<0.01){
        is_vertical=true;
        //        LOG(INFO)<<"垂直线";
    }else if( fabs(fabs(phi)-90)<0.01
              || fabs(fabs(phi)-270)<0.01){
        is_horizontal=true;
        //          LOG(INFO)<<"水平线";
    }
}

bool StraightLine::GetY(float x, float &y)
{
    if(this->is_vertical){
        return false;
    }else if(this->is_horizontal){
        y=this->rho;
        return true;
    }else{
        y=this->k*x+this->b;
        return true;
    }

}

bool StraightLine::GetX(float y, float &x)
{
    //水平线，取不到指定的y对应的x值，因为y值永远不变
    if(this->is_horizontal){
        x=0;
        return false;
    }else if(this->is_vertical){
        x=this->rho;
        return true;
    }else{
        x=(y-b)/k;
        return true;
    }
}

int StraightLine::GetInterceptPoint(StraightLine &another,cv::Point2f& pt)
{
    if(this->is_valid==false || another.is_valid==false){
        return 0;
    }

    //----both vertical or both horizontal----//
    if(this->is_vertical && another.is_vertical){
        if(fabs(fabs(this->rho) - fabs(another.rho))<0.0001){
            return 1;
        }else{
            return 0;
        }
    }else if(this->is_horizontal && another.is_horizontal){
        if(fabs(fabs(b)-fabs(another.b))<0.0001){
            return  1 ;
        }else{
            return false;
        }
    }


    //--------one is vertical and another is horizontal----//
    if(this->is_vertical && another.is_horizontal){
        pt.x=rho;
        pt.y=another.b;

        return 2;
    }

    if(this->is_horizontal && another.is_vertical){
        pt.x=another.rho;
        pt.y=this->b;

        return 2;
    }


    //----------other case ------//
    pt.x=(another.b-this->b)/(this->k-another.k);
    pt.y=this->k * pt.x + this->b;



    return 2;
}

float StraightLine::GetDistanceToPoint(Point2f &pt)
{
    if(this->is_valid==false){
        return 999999;
    }
    if(this->is_horizontal){
        return fabs(pt.y-b);
    }else if(this->is_vertical){
        return fabs(pt.x-rho);
    }else{
        return fabs(pt.y-k*pt.x-b)/sqrt(1+k*k);
    }
}

void StraightLine::Draw(cv::Mat& pict,cv::Scalar color,int thickness){


    if(is_vertical){
        cv::Point p1(rho,0);
        cv::Point p2(rho,pict.rows);

        cv::line(pict,p1,p2,color,thickness);
    }else if(is_horizontal){
        cv::Point p1(0,rho);
        cv::Point p2(pict.cols,rho);

        cv::line(pict,p1,p2,color,thickness);
    }else {
        float x,y;
        bool isok=GetX(0,x);

        float x2;
        isok=GetX(pict.rows,x2);
        cv::Point p1(x,0);
        cv::Point p2(x2,pict.rows);

        cv::line(pict,p1,p2,color,thickness);
    }

}

void StraightLine::Print()
{
    std::cout<<(is_vertical?"vertical line":(is_horizontal?"horizontal line.":"normal line"))<<endl;
    std::cout<<"k="<<k<<",b="<<b<<endl;
}

void CandidateGroup::Print()
{
    cout<<"group info (x_btm):";
    for(LaneMarkingSeg* seg:  this->lane_marks){
        cout<<seg->x_btm<<" ";
    }
    cout<<endl;
}

bool CandidateGroup::IsExist(LaneMarkingSeg *one)
{
    for(LaneMarkingSeg* each:this->lane_marks){
        if(one==each){
            return true;
        }
    }

    return false;
}

RoadPaintMarking::RoadPaintMarking()
{
    RNG rng(clock());
    draw_color=cv::Scalar(rng.uniform(0,rng.uniform(100,255)),rng.uniform(0,255),rng.uniform(0,255));
}

void RoadPaintMarking::AddCandidateSupportSeg(LaneMarkingSeg &seg)
{
    this->candidate_segs.push_back(&seg);
}

void RoadPaintMarking::ClearCandidateSegs()
{
    this->candidate_segs.clear();
}

void RoadPaintMarking::AddSupportSeg(LaneMarkingSeg &seg,long seq)
{
    last_frame_seq_support=seq;
    support_history_segs.push_back(seg);

    confidence+=seg.dir_len;

}




}//end of namespace gumh
