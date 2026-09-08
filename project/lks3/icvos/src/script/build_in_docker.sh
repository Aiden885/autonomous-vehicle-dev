#! /bin/bash
set +e
SCRIPTDIR=$(dirname "${BASH_SOURCE[0]}")
source $SCRIPTDIR/config.bash
BASHARGS=""
DOCKERNAME=$(date "+%Y%m%d%H%M%S")
VOLUMEDIR="-v $WORKDIR:$WORKDIR"
begin_time=$(date +%s) # 获取当前时间戳
UPDATE_DOCKER=0
UPDATE_ICVOS_3RD=0 # 是否动态获取最新版本 1:动态获取最新 0:使用指定版本
function build_mods() {

    ATH="sudo"
    if [ "${USERATH}" = "root" ]; then
        ATH=""
    fi

    # $ATH chown -R $USER:$USER $WORKDIR || true

    check_docker_tag

    if [ $UPDATE_DOCKER -ne 0 ]; then
      update_docker_image ${COMPILE_DOCKER}${DOCKERPREFIX}:${DOCKERTAG} $ATH
    fi

    DOCKERNAME="${DOCKERNAME}${DOCKERPREFIX}_${RANDOM}"
    DOCKERNAME=${DOCKERNAME/\//-}
    echo "DOCKERNAME: $DOCKERNAME"
    $ATH docker run --rm --name=$DOCKERNAME ${VOLUMEDIR} ${COMPILE_DOCKER}${DOCKERPREFIX}:${DOCKERTAG} /bin/bash -c "export PS1=1 && source /root/.bashrc && cd $WORKDIR && bash ./script/build_local.sh $BASHARGS $@"
    build_res=$?
    $ATH chown -R $USER:$USER $WORKDIR || true
    end_time=$(date +%s)            # 获取当前时间戳
    diff=$((end_time - begin_time)) # 计算时间差，单位为秒
    if [ $build_res -ne 0 ]; then
        echo -e "${RED}编译失败!!${END}"
        exit 3
    else
        echo -e "$GREEN编译成功!!用时${diff}秒$END"
    fi
}

while [ "$1" != "" ]; do
    PARAM=$(echo $1 | awk -F '[:=]' '{print $1}')
    VALUE=$(echo $1 | awk -F '[:=]' '{print $2}')
    case $PARAM in
    -arch | -a | --a | --arch)
        if [ "$VALUE" = "aarch64" ] || [ "$VALUE" = "arm" ]; then
            DOCKERPREFIX="-aarch64"
            BASHARGS="$BASHARGS -a:aarch64"
            echo "build for aarch64..."
        elif [ "$VALUE" = "mdc510" ] || [ "$VALUE" = "mdc" ]; then
            DOCKERPREFIX="mdc510"
            BASHARGS="$BASHARGS -a:mdc510"
            echo "build for mdc 510..."
        elif [ "$VALUE" = "mdc610" ]; then
            DOCKERPREFIX="mdc610"
            BASHARGS="$BASHARGS -a:mdc610"
            echo "build for mdc 610..."
        elif [ "$VALUE" = "j6" ]; then
            DOCKERPREFIX="j6"
            BASHARGS="$BASHARGS -a:j6"
            echo "build for j6..."
        elif [ "$VALUE" = "c1200" ]; then
            DOCKERPREFIX="c1200"
            BASHARGS="$BASHARGS -a:c1200"
            echo "build for c1200..."
        else
            BASHARGS="$BASHARGS -a:x86"
            echo "build for x86..."
        fi
        ;;
    -r | -root | --r | --root)
        USERATH="root"
        BASHARGS="-root $BASHARGS"
        ;;
    -name | --name | -n | --n)
        DOCKERNAME="${VALUE}_${DOCKERNAME}"
        ;;
    -tag | --tag)
        DOCKERTAG="${VALUE}"
        ;;

    -v | --v)
        VOLUMEDIR="${VOLUMEDIR} -v ${VALUE}:${VALUE}"
        ;;
    -ud)
        UPDATE_DOCKER=1
        ;;
    -dep_update)
        UPDATE_ICVOS_3RD=1
        ;;
    *)
        BASHARGS="$BASHARGS $1"
        ;;
    esac
    shift
done

bash $SCRIPTDIR/ftp/download_icvos_3rdparty.sh $DOCKERPREFIX $UPDATE_ICVOS_3RD
if [ $? -ne 0 ]; then
  exit 3
fi

build_mods
