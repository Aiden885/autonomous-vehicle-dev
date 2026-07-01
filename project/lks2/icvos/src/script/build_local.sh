#! /bin/bash

set +e
SCRIPTDIR=$(dirname "${BASH_SOURCE[0]}")
source $SCRIPTDIR/config.bash

if [ $# = 0 ]; then
    _usage
    exit 3
fi

while [ "$1" != "" ]; do
    PARAM=$(echo $1 | awk -F '[:=]' '{print $1}')
    VALUE=$(echo $1 | awk -F '[:=]' '{print $2}')
    case $PARAM in
    -ut | --ut)
        if [ "$VALUE" = "off" ] || [ "$VALUE" = "no" ]; then
            echo "build without unit test..."
            BUILDARGS="$BUILDARGS -DUTEST=off"
        else
            UTEST=1
            BUILDARGS="$BUILDARGS -DUTEST=on"
            echo "build with unit test..."
        fi
        ;;
    -u | -update | --u | --update)
        UPDATEDEP=1
        if [ "$VALUE" != "" ]; then
            CODE_BRANCH="origin $VALUE"
            echo $CODE_BRANCH
        fi
        ;;
    -arch | -a | --a | --arch)
        if [ "$VALUE" = "aarch64" ] || [ "$VALUE" = "arm" ]; then
            echo "build for aarch64..."
            ENVIRONMENT="aarch64"
            ROS_OPT="noROS"
            THIRD_PATH="${WORKDIR}/dependencies/thirdparty/aarch64"
            BUILDARGS="$BUILDARGS -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc -DARCH=aarch64"
        elif [ "$VALUE" = "mdc510" ]; then
            echo "build for mdc-510.."
            ENVIRONMENT="mdc510"
            ROS_OPT="noROS"
            THIRD_PATH="${WORKDIR}/dependencies/thirdparty/MDC510"
            BUILDARGS="$BUILDARGS -DCMAKE_CXX_COMPILER=/opt/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/bin/clang++ -DCMAKE_C_COMPILER=/opt/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/bin/clang -DARCH=aarch64 -DBUILD_TARGET=MDC_510"
        elif [ "$VALUE" = "mdc610" ]; then
            echo "build for mdc-610..."
            ENVIRONMENT="mdc610"
            ROS_OPT="noROS"
            BUILD_PLATFORM="MDC610"
            THIRD_PATH="${WORKDIR}/dependencies/thirdparty/MDC610"
            BUILDARGS="$BUILDARGS -DCMAKE_CXX_COMPILER=/opt/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/bin/clang++ -DCMAKE_C_COMPILER=/opt/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/bin/clang -DARCH=aarch64 -DBUILD_TARGET=MDC_610"
        elif [ "$VALUE" = "j6" ]; then
            echo "build for j6..."
            ENVIRONMENT="j6"
            ROS_OPT="noROS"
            THIRD_PATH="${WORKDIR}/dependencies/thirdparty/J6"
            BUILDARGS="$BUILDARGS -DCMAKE_TOOLCHAIN_FILE=/opt/toolchain.cmake -DARCH=aarch64 -DBUILD_TARGET=J6"
        elif [ "$VALUE" = "c1200" ]; then
            echo "build for c1200..."
            ENVIRONMENT="c1200"
            ROS_OPT="noROS"
            THIRD_PATH="${WORKDIR}/dependencies/thirdparty/C1200"
            BUILDARGS="$BUILDARGS -DCMAKE_TOOLCHAIN_FILE=/opt/toolchain.cmake -DARCH=aarch64 -DBUILD_TARGET=C1200"
        else
            ENVIRONMENT="x86"
            BUILD_PLATFORM="X86"
            ROS_OPT="ROS2"
            echo "build for x86...$ROS_OPT"
        fi
        ;;
    -host | -host_platform | --host | --host_platform)
        if [ "$VALUE" = "aarch64" ] || [ "$VALUE" = "arm" ]; then
            echo "building system is aarch64..."
            BUILDARGS="$BUILDARGS -DHOST_PLATFORM=aarch64"
            HOST_PF="aarch64"
            PROTODIR="${WORKDIR}/dependencies/thirdparty/aarch64/lib"
        elif [ "$VALUE" = "mdc510" ]; then
            echo "building system is mdc510..."
            BUILDARGS="$BUILDARGS -DHOST_PLATFORM=mdc510"
            HOST_PF="mdc510"
            PROTODIR="${WORKDIR}/dependencies/thirdparty/mdc510/lib"
        elif [ "$VALUE" = "mdc610" ]; then
            echo "building system is mdc610..."
            BUILDARGS="$BUILDARGS -DHOST_PLATFORM=mdc610"
            HOST_PF="mdc610"
            PROTODIR="${WORKDIR}/dependencies/thirdparty/mdc610/lib"
        else
            echo "building system is x86..."
        fi
        ;;
    -r | -root | --r | --root)
        USERATH="root"
        UTAGS="root $UTAGS"
        ;;
    -d | -debug | --d | --debug)
        BUILDARGS="$BUILDARGS -DENABLE_DEBUG=on"
        ;;
    -b | -build | --b | --build)
        BUILD=1
        ;;
    -c | -clear | --c | --clear)
        CLEAR=1
        ;;
    -ros | -ROS | --ros | --ROS)
        BUILDARGS="$BUILDARGS -DROS_VER=$VALUE"
        if [ "$VALUE" = "2" ]; then
            ROS_OPT="ROS2"
            echo "build with ros2 ..."
        elif [ "$VALUE" = "1" ]; then
            ROS_OPT="ROS1"
            echo "build with ros1 ..."
        else
            ROS_OPT="noROS"
            echo "build without ros..."
        fi
        ;;
    -common_message | -COMMON_MESSAGE | -common_msg | -msg | -COMMON_MSG | -MSG | --common_message | --COMMON_MESSAGE | --common_msg | --msg | --COMMON_MSG | --MSG)
        BUILDARGS="$BUILDARGS -DCOMMON_MESSAGE=$VALUE"
        ;;
    -user | --user)
        BUILDARGS="$BUILDARGS $VALUE"
        ;;
    -pkg | --pkg)
        shift
        WHITE_LIST=""
        if [ "$1" != "" ] && [[ $1 != -* ]]; then
            BUILDARGS="$BUILDARGS -DOPEN_WHITE_LIST=on"
        else
            BUILDARGS="$BUILDARGS -DOPEN_WHITE_LIST=off"
        fi
        while [ "$1" != "" ] && [[ $1 != -* ]]; do
            echo -e "${GREEN}Add module[$1] in build white list!$END"
            if [ -z "$(find "${WORKDIR}/modules" -type d -name $1)" ]; then
                echo -e "${RED}modules下没有模块目录$1$END"
                exit -1
            fi
            WHITE_LIST="$WHITE_LIST;$1:"
            shift
        done
        BUILDARGS="$BUILDARGS -DWHITE_LIST=\"$WHITE_LIST\""
        continue
        ;;
    -no_pkg | --no_pkg)
        shift
        BLACK_LIST=""
        while [ "$1" != "" ] && [[ $1 != -* ]]; do
            echo -e "${YELLOW}Add module[$1] in build black list!$END"
            if [ -z "$(find "${WORKDIR}/modules" -type d -name $1)" ]; then
                echo -e "${RED}modules下没有模块目录$1$END"
                exit -1
            fi
            BLACK_LIST="$BLACK_LIST;$1:"
            shift
        done
        BUILDARGS="$BUILDARGS -DBLACK_LIST=\"$BLACK_LIST\""
        continue
        ;;
    -o | --os | -runtime_os | --runtime_os)
        echo -e "${GREEN}RUNTIME_OS set be ICVOS!$END"
        RUNTIME_OS="ICVOS"
        BUILDARGS="$BUILDARGS -DRUNTIME_OS=ICVOS"

        ;;
    -s | --s | -shadow | --shadow)
        BUILDARGS="$BUILDARGS -DBUILD_SHADOW=on"
        ;;
    -t | --t | -thirdparty | --thirdparty)
        BUILDARGS="$BUILDARGS -DINSTALL_THIRDPARTY_LIB=$VALUE"
        ;;
    -l | --l | -load | --load_backup)
        target_env="$VALUE"
        echo "target_env is ${SUPPORT_ENV[${target_env}]}"
        if [ -z "${SUPPORT_ENV[${target_env}]}" ]; then
            echo -e "${RED}加载的目标环境为'${target_env}'，目前仅支持'${!SUPPORT_ENV[@]}'！${END}"
        else
            load_backup ${target_env}
        fi
        ;;
    -j)
        BUILD_JOBS=$VALUE
        echo "BUILD_JOBS is ${BUILD_JOBS}"
        ;;
    *)
        _usage
        exit
        ;;

    esac
    shift
done

#集成编译 extension 所需
source ${WORKDIR}/dependencies/thirdparty/${BUILD_PLATFORM}/setup.bash
source ${WORKDIR}/dependencies/common/os_adapter/icvos/${BUILD_PLATFORM}/sdk/setup.bash
THIRDPARTY_INSTALL=${WORKDIR}/dependencies/thirdparty/${BUILD_PLATFORM}
if [[ "${BUILD_PLATFORM}" == "MDC610" ]]; then
  if [ -d "${THIRDPARTY_INSTALL}/lib/python3.7/site-packages/numpy" ]; then
    rm -rf ${THIRDPARTY_INSTALL}/lib/python3.7/site-packages/numpy_
    mv ${THIRDPARTY_INSTALL}/lib/python3.7/site-packages/numpy ${THIRDPARTY_INSTALL}/lib/python3.7/site-packages/numpy_
  fi
fi

if [ $CLEAR -ne 0 ]; then
    clear_temp
fi
if [ $? -ne 0 ]; then
    echo -e "${RED}clear failed!!!$END"
    exit -6
fi

if [ $UPDATEDEP -ne 0 ]; then
    update_dep "${CODE_BRANCH}"
fi
if [ $? -ne 0 ]; then
    echo -e "${RED}更新代码仓和依赖库失败!!$END"
    exit -6
fi

if [ $BUILD -ne 0 ]; then
    checkenv $WORKDIR
    build $WORKDIR
fi

if [ $? -ne 0 ]; then
    echo "编译失败!!"
    exit -6
fi
