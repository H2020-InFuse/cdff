#!/usr/bin/env bash
# will use the Dockerfile provided, check it against current last version of it
# Will push it to Nexus if the dockerfile contains a new label with imagename::LABEL
# eg Usage docker-builder.sh Dockerfile.base cdff-ci
# will produce nexus.spaceapplications.com/repository/infuse/cdff-ci:latest and '...'cdff-ci:LABEL

set -e

if [ $# -lt 2 ]; then
    echo "incorrect parameters: Usage docker-builder.sh [dockerfile] [imagename] [optional=--force] [optional=environnement variables]"
    exit 1
fi

# will return $op as '<' '=" or '>'
vercomp () {
    if [[ $1 == $2 ]]
    then
        op='=';
        return
    fi
    local IFS=.
    local i ver1=($1) ver2=($2)
    # fill empty fields in ver1 with zeros
    for ((i=${#ver1[@]}; i<${#ver2[@]}; i++))
    do
        ver1[i]=0
    done
    for ((i=0; i<${#ver1[@]}; i++))
    do
        if [[ -z ${ver2[i]} ]]
        then
            # fill empty fields in ver2 with zeros
            ver2[i]=0
        fi
        if ((10#${ver1[i]} > 10#${ver2[i]}))
        then
            op='>';
            return
        fi
        if ((10#${ver1[i]} < 10#${ver2[i]}))
        then
            op='<';
            return
        fi
    done
    op='=';
    return
}

IMAGE_NAME=$2
DOCKER_FILE=$1

#if we passed env variables we pass them along
ENV_VAR=""
if ! test -z "$4"
then
   ENV_VAR='--build-arg '$4' '
fi

REGISTRY_PREFIX=nexus.spaceapplications.com/
INFUSE_REGISTRY_PREFIX=repository/infuse/
IMAGE_TAG=${REGISTRY_PREFIX}${INFUSE_REGISTRY_PREFIX}${IMAGE_NAME}

current_tag=$(grep "LABEL version=" $DOCKER_FILE | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/')
latest_tag=$(curl -d 'hook_private_key=ea9dc697-5bc9-4a43-96aa-6257f2fda70e&key='$IMAGE_NAME https://hook.io/datastore/get | tr -d '"')

if [ -z "$latest_tag" ] || [ $latest_tag == 'null' ] ; then
    latest_tag="0.0"
fi

if [ -z "$current_tag" ]; then
    current_tag="0.0"
fi

echo latest_tag $latest_tag
echo current_tag $current_tag
vercomp $current_tag $latest_tag

if [[ $op = '>' ]] || [[ $3 == "--force" ]]
    then
        docker pull $IMAGE_TAG':'$latest_tag || true
        docker build -t $IMAGE_TAG':'$current_tag -f $DOCKER_FILE $ENV_VAR .
        docker push $IMAGE_TAG':'$current_tag
        docker tag $IMAGE_TAG':'$current_tag $IMAGE_TAG':'latest
        docker push $IMAGE_TAG':'latest
    curl -d 'hook_private_key=ea9dc697-5bc9-4a43-96aa-6257f2fda70e&key='$IMAGE_NAME'&value='$current_tag https://hook.io/datastore/set
    else
        echo Image $IMAGE_TAG':'$latest_tag already available on the server.
fi
