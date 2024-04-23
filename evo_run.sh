#!/bin/bash

trap "echo -e \"Oops,evo running interruption...\";exit" INT

# 定义所有可能的参数值
args_a=("0" "1")
args_b=("~/Data/datasets/rgbd_dataset_freiburg2_desk" \
        "~/Data/datasets/" \
        "~/Data/datasets/" \
        "~/Data/datasets/" \
        "~/Data/datasets/")

# 循环遍历所有参数组合
for a in "${args_a[@]}"; do
  for b in "${args_b[@]}"; do
    echo "Running mono_tum with arguments: a=$a, b=$b"
    runORB $a $b
    echo "========================="
  done
done

function runORB() {
    disCheckEN=$1
    datesetPath=$2
    # 使用字符串处理功能提取数字部分
    # 使用basename命令获取路径中的最后一部分
    filename=$(basename "$datesetPath")

    # 使用正则表达式提取数字部分
    # 这里的正则表达式含义是：匹配字符串中的数字部分（可能是多个数字）
    tum_fr=0
    if [[ $filename =~ [0-9]+ ]]; then
        tum_fr="${BASH_REMATCH[0]}"
        echo "从路径中提取的数字为: $tum_fr"
    else
        echo "未找到匹配的数字"
    fi

    now_time=$(date +'%Y-%m-%d_%H-%M-%S')
    echo -n "test time is: = $now_time" > evo_result.txt
    echo " datesetPath = ${datesetPath} disCheckEN = ${disCheckEN}" >> evo_result.txt

    mkdir ./floor_results/${filename}/${disCheckEN}/${now_time}

    i=1
    until [ $i -gt 30 ]
    do
        #run slam code
        ./Examples/Monocular/mono_tum ./Vocabulary/ORBvoc.txt ./Examples/Monocular/TUM${tum_fr}.yaml ${datesetPath} ./floor_results/${filename}/${disCheckEN}/${now_time}/Log/mse_log_${i}.txt ${disCheckEN}
        cp -f KeyFrameTrajectory.txt ./floor_results/${filename}/${disCheckEN}/${now_time}/KeyFrameTrajectory_$i.txt
        rm -rf ./evo_output
        mkdir evo_output
        echo "--------------------$i-----------------------" >> evo_result.txt
        #使用evo_ape 测量误差
        path=$(date +'%Y_%m_%d_%H_%M_%S')
        evo_ape tum $datesetPath/groundtruth.txt KeyFrameTrajectory.txt  -va --save_results ./evo_output/ape_$path

        #解压并保存rmse数据
        unzip -o ./evo_output/ape_$path -d ./evo_output

        echo -n "evo_ape rmse: = " >> evo_result.txt
        cat ./evo_output/stats.json | jq '.rmse' >> evo_result.txt

        #使用evo_rpe 测量误差 
        path=$(date +'%Y_%m_%d_%H_%M_%S')
        evo_rpe tum $datesetPath/groundtruth.txt KeyFrameTrajectory.txt  -va --save_results ./evo_output/rpe_$path
        unzip -o ./evo_output/rpe_$path -d ./evo_output

        echo -n "evo_rpe rmse: = " >> evo_result.txt
        cat ./evo_output/stats.json | jq '.rmse' >> evo_result.txt
        i=$((i+1))
    done

    cp -f evo_result.txt ./floor_results/${filename}/${disCheckEN}/${now_time}/evo_result.txt
}

