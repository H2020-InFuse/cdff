#!/usr/bin/env bash
# will use the Dockerfile provided, check it against current last version of it
# Will push it to Nexus if the dockerfile contains a new label with imagename::LABEL
# eg Usage docker-builder.sh Dockerfile.ci cdff-ci
# will produce nexus.spaceapplications.com/repository/infuse/cdff-ci:latest and '...'cdff-ci:LABEL


if [ $# < 2 ]; then
    echo "incorrect parameters: Usage docker-builder.sh [dockerfile] [imagename] [optional=environnement variables]"
    exit 1
fi

IMAGE_NAME=$2
DOCKER_FILE=$1

#if we passed env variables we pass them along
ENV_VAR=""
if test -z "$3"
then
   ENV_VAR='-e '$3' '
fi

REGISTRY_PREFIX=nexus.spaceapplications.com/
INFUSE_REGISTRY_PREFIX=repository/infuse/
IMAGE_TAG=${REGISTRY_PREFIX}${INFUSE_REGISTRY_PREFIX}${IMAGE_NAME}

current_tag=$(grep "LABEL version=" $DOCKER_FILE | perl -pe '($_)=/([0-9]+([.][0-9]+)+)/')
latest_tag=$(curl -d 'hook_private_key=ea9dc697-5bc9-4a43-96aa-6257f2fda70e&key='$IMAGE_NAME https://hook.io/datastore/get | tr -d '"')
echo latest_tag $latest_tag , current_tag  $current_tag

if [[ ! $latest_tag < $current_tag ]]; then 
    login -u $DOCKER_USER -p $DOCKER_PASSWORD $REGISTRY_PREFIX
    docker pull $IMAGE_TAG':'$latest_tag
    docker build -t $IMAGE_TAG':'$current_tag -f $DOCKER_FILE $ENV_VAR .
    docker push $IMAGE_TAG':'$current_tag
    docker tag $IMAGE_TAG':'$current_tag $IMAGE_TAG':'latest
    docker push $IMAGE_TAG':'latest
    curl -d 'hook_private_key=ea9dc697-5bc9-4a43-96aa-6257f2fda70e&key='$IMAGE_NAME'&value='$current_tag https://hook.io/datastore/set
 else 
 echo Image $IMAGE_TAG':'$latest_tag already available on the server."
fi 

 
 
