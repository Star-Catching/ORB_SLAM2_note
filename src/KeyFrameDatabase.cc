/**
 * @file KeyFrameDatabase.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 关键帧数据库,用于回环检测和重定位
 * @version 0.1
 * @date 2019-04-25
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

// 构造函数
KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    // 数据库的主要内容了
    mvInvertedFile.resize(voc.size()); // number of words
}

// 根据关键帧的词包，更新数据库的倒排索引
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    // 为每一个word添加该KeyFrame
    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

// 关键帧被删除后，更新数据库的倒排索引
void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    // 每一个KeyFrame包含多个words，遍历mvInvertedFile中的这些words，然后在word中删除该KeyFrame
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word

        list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];
        // 这个效率有点低啊
        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

// 清空关键帧数据库
void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();// mvInvertedFile[i]表示包含了第i个word id的所有关键帧
    mvInvertedFile.resize(mpVoc->size());// mpVoc：预先训练好的词典
}

/*
 * @brief 在闭环检测中找到与该关键帧可能闭环的关键帧
 * 1. 找出和当前帧具有公共单词的所有关键帧（不包括与当前帧相连的关键帧）
 * 2. 只和具有共同单词较多的关键帧进行相似度计算
 * 3. 将与关键帧相连（权值最高）的前十个关键帧归为一组，计算累计得分
 * 4. 只返回累计得分较高的组中分数最高的关键帧
 * @param pKF      需要闭环的关键帧
 * @param minScore 相似性分数最低要求
 * @return         可能闭环的关键帧
 * @see III-E Bags of Words Place Recognition
 */
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    // 提出所有与该pKF相连的KeyFrame，这些相连Keyframe都是局部相连，在闭环检测的时候将被剔除
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWords;// 用于保存可能与pKF形成回环的候选帧（只要有相同的word，且不属于局部相连帧）
    //这里的局部相连帧,就是和当前关键帧具有共视关系的关键帧

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    // step1：找出和当前帧具有公共单词的所有关键帧（不包括与当前帧链接的关键帧）
    {
        unique_lock<mutex> lock(mMutex);

        // words是检测图像是否匹配的枢纽，遍历该pKF的每一个word
        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            // 提取所有包含该word的KeyFrame
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
            // 然后对这些关键帧展开遍历
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)// pKFi还没有标记为pKF的候选帧
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))// 与pKF局部链接的关键帧不进入闭环候选帧
                    {
                        pKFi->mnLoopQuery=pKF->mnId;// pKFi标记为pKF的候选帧，之后直接跳过判断
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;// 记录pKFi与pKF具有相同word的个数
            }
        }
    }

    // 如果没有关键帧和这个关键帧具有相同的单词,那么就返回空
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    // step2：统计所有闭环候选帧中与pKF具有共同单词最多的单词数
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    // step3：遍历所有闭环候选帧，挑选出共有单词数大于minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        // pKF只和具有共同单词较多的关键帧进行比较，需要大于minCommonWords
        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;// 这个变量后面没有用到

            // 相似度评分就是在这里计算的（利用mBowVec来计算两者的相似度得分）
            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    // 如果没有超过指定相似度阈值的，那么也就直接跳过去
    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();


    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
    // step4：具体而言：lScoreAndMatch中每一个KeyFrame都把与自己共视程度较高的帧归为一组，每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatch
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first; // 该组最高分数
        float accScore = it->first;  // 该组累计得分
        KeyFrame* pBestKF = pKFi;    // 该组最高分数对应的关键帧
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;// 因为pKF2->mnLoopQuery==pKF->mnId，所以只有pKF2也在闭环候选帧中，才能贡献分数
                if(pKF2->mLoopScore>bestScore)// 统计得到组里分数最高的KeyFrame
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)// 记录所有组中组得分最高的组 用于确定相对阈值
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    // step5：得到组得分大于minScoreToRetain的组，得到组中分数最高的关键帧 0.75*bestScore
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))// 判断该pKFi是否已经在队列中了
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpLoopCandidates;
}

/*
 * @brief 在重定位中找到与该帧相似的关键帧
 * 1. 找出和当前帧具有公共单词的所有关键帧
 * 2. 只和具有共同单词较多的关键帧进行相似度计算
 * 3. 将与关键帧相连（权值最高）的前十个关键帧归为一组，计算累计得分
 * 4. 只返回累计得分较高的组中分数最高的关键帧
 * @param F 需要重定位的帧
 * @return  相似的关键帧
 * @see III-E Bags of Words Place Recognition
 */
// NOTE 但是我觉得基本上的套路和进行回环检测的时候基本上行都是相同的
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    // 相对于关键帧的闭环检测DetectLoopCandidates，重定位检测中没法获得相连的关键帧
    list<KeyFrame*> lKFsSharingWords;// 用于保存可能与F形成回环的候选帧（只要有相同的word，且不属于局部相连帧(这里其实已经没有了所谓的"局部相连帧"的概念了)）

    // Search all keyframes that share a word with current frame
    //. 步骤1：找出和当前帧具有公共单词的所有关键帧
    //hwh Step 1.利用逆序索引mnRelocWords 记录之前的某一关键帧与当前帧存在相同特征点word的数目mnRelocWords
    // wxz 找出和当前帧具有公共单词（word）的所有关键帧
    {
        unique_lock<mutex> lock(mMutex);

        // words是检测图像是否匹配的枢纽，遍历该pKF的每一个word
        // wxz mBowVec 内部实际存储的是std：：map<WordId,WordValue>
        // wxz WordId和WordValue 表示Word在叶子中的id 和权重 
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            // 提取所有包含该word的KeyFrame
            list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];  // wxz 倒排索引

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId)// pKFi还没有标记为pKF的候选帧  
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                //hwh mnRelocWords变量表示之前的某一关键帧与当前帧存在相同特征点word的数目
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    // Only compare against those keyframes that share enough words
    //. 步骤2：统计所有闭环候选帧中与当前帧F具有共同单词最多的单词数，并以此决定阈值
    //hwh Step2 找出最大的mnRelocWords，并以最大的mnRelocWords的80％(minCommonWords)作为划分阈值
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    //. 步骤3：遍历所有闭环候选帧，挑选出共有单词数大于阈值minCommonWords且单词匹配度大于minScore存入lScoreAndMatch
    //hwh Step3 利用TF-IDF权重计算候选帧(mnRelocWords大于minCommonWords的候选帧)与当前帧的得分
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        // 当前帧F只和具有共同单词较多的关键帧进行比较，需要大于minCommonWords
        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;// 这个变量后面没有用到
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    // 步骤4：计算候选帧组得分，得到最高组得分bestAccScore，并以此决定阈值minScoreToRetain
    // 单单计算当前帧和某一关键帧的相似性是不够的，这里将与关键帧相连（权值最高，共视程度最高）的前十个关键帧归为一组，计算累计得分
    // 具体而言：lScoreAndMatch中每一个KeyFrame都把与自己共视程度较高的帧归为一组，每一组会计算组得分并记录该组分数最高的KeyFrame，记录于lAccScoreAndMatch
    //hwh Step4 在step3滤除后的候选帧中，找到每一个候选帧的共视关键帧组(1个候选帧有10个共视关键帧，一共11个关键帧)组成每一个候选帧组
    //hwh       计算候选帧组中mRelocScore的总得分accScore，并找到mnRelocQuery得分最高的候选帧pBestKF
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first; // 该组最高分数
        float accScore = bestScore;  // 该组累计得分
        KeyFrame* pBestKF = pKFi;    // 该组最高分数对应的关键帧
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore;// 只有pKF2也在闭环候选帧中，才能贡献分数
            if(pKF2->mRelocScore>bestScore)// 统计得到组里分数最高的KeyFrame
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore) // 记录所有组中组得分最高的组
            bestAccScore=accScore; // 得到所有组中最高的累计得分
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    // 步骤5：得到组得分大于阈值的，组内得分最高的关键帧
    //hwh Step5 在得分大于阈值的候选帧组中找到组内得分最高的关键帧作为最终候选帧
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        // 只返回累计得分大于minScoreToRetain的组中分数最高的关键帧 0.75*bestScore
        if(si>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))// 判断该pKFi是否已经在队列中了
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

} //namespace ORB_SLAM
