#!/bin/bash

# Maintainers: sco@spaceapplications.com

#echo "========================================="
#echo "This is a message from the Jenkins Docker!"

# curl --silent --show-error --connect-timeout 1 -I ${JENKINS_SERVER}/job/valgrind-report-generate/api/xml

DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
JENKINS_SERVER="http://localhost:8080"
#JENKINS_SERVER="http://nexus.spaceapplications.com-repository-infuse-cdff-jenkins:8080"
INPUT_FILE=${1:-"${DIR}/report/valgrind.xml"}
OUTPUT_FOLDER=${2:-"${DIR}/report"}

counter=0
MAX_RETRIES=20

echo "Trying to connect to the local Jenkins instance..."
until [ "`curl --silent --show-error --connect-timeout 1 -I ${JENKINS_SERVER}/job/valgrind-report-generate/api/xml | grep 'HTTP/1.1 200 OK'`" != "" ];
do
  if [ $counter -gt $MAX_RETRIES ]
  then
  echo "Took too long to connect to localhost -- assuming something went wrong! ABORTING"
  exit 1
  fi
  echo    --- sleeping for 5 second
  sleep 5
  counter=$((counter+1))
done

echo "===================================="
echo "Jenkins is now fully up and running!"
echo "===================================="

mkdir -p /var/jenkins_home/jobs/valgrind-report-generate/workspace/
cp ${INPUT_FILE} /var/jenkins_home/jobs/valgrind-report-generate/workspace/valgrind.xml
sleep 5
echo "Executing job/valgrind-report-generate"

#curl -X POST ${JENKINS_SERVER}/job/valgrind-report-generate/buildWithParameters?token=valgrind \
#  --show-error --user admin:admin \
#  --form file0=@${INPUT_FILE} \
#  --form json='{"parameter": [{"name":"valgrind.xml", "file":"file0"}]}'

curl --silent --show-error --connect-timeout 1 -I ${JENKINS_SERVER}/job/valgrind-report-generate/buildWithParameters?token=valgrind

counter=0
sleep 5
echo "Pinging for result of Valgrind report generation"
until [ "`curl --silent --show-error --connect-timeout 1 -I ${JENKINS_SERVER}/job/valgrind-report-generate/lastBuild/api/json | grep 'HTTP/1.1 200 OK'`" != "" ];
do
  if [ $counter -gt $MAX_RETRIES ]
  then
  echo "Took too long to find a lastBuild -- assuming something went wrong! ABORTING"
  exit 1
  fi

  echo    --- No lastBuild JSON available yet - sleeping for 5 second
  sleep 5
  counter=$((counter+1))
done

counter=0
echo "===== lastBuild info is available ====="
sleep 5
echo "Pinging for finishing of Valgrind report generation"
until [ "`curl --silent --show-error --connect-timeout 1 ${JENKINS_SERVER}/job/valgrind-report-generate/lastBuild/api/json | jq -r '.building'`" == "false" ];
do
  if [ $counter -gt $MAX_RETRIES ]
  then
  echo "Took too long for build to finish -- assuming something went wrong! ABORTING"
  exit 1
  fi

  echo    --- Not finished building yet - sleeping for 20 seconds
  sleep 20
  counter=$((counter+1))
done

echo "===== lastBuild has finished ====="
# result can be SUCCESS | FAILURE
echo "Dumping the console output of the Jenkins job into a file for safe storage!"
curl --silent --connect-timeout 1 ${JENKINS_SERVER}/job/valgrind-report-generate/lastBuild/consoleText > ${OUTPUT_FOLDER}/valgrind_report_generate_consoleText.log
if [ "`curl --silent --show-error --connect-timeout 1 ${JENKINS_SERVER}/job/valgrind-report-generate/lastBuild/api/json | jq -r '.result'`" == "SUCCESS" ]
then
echo "Valgrind report generation successful! Static HTML generation should start automatically!"
else
echo "Valgrind report generaton was a failure! Probably because of an error with valgrind.xml"
echo "The static html generation build will not be triggered (or even relevant) so aborting!"
exit 1
fi

# gets the status of the build
#curl --silent --show-error --connect-timeout 1 ${JENKINS_SERVER}/job/valgrind-report-generate/lastBuild/api/json | jq -r '.result'

# gets the result of the build
#curl --silent --show-error --connect-timeout 1 ${JENKINS_SERVER}/job/valgrind-report-generate/lastBuild/api/json | jq -r '.building'


sleep 5
echo "Pinging for result of static html generation"
counter=0
until [ "`curl --silent --show-error --connect-timeout 1 -I ${JENKINS_SERVER}/job/generate-static-html/lastBuild/api/json | grep 'HTTP/1.1 200 OK'`" != "" ];
do
  if [ $counter -gt $MAX_RETRIES ]
  then
  echo "Took too long to find a lastBuild -- assuming something went wrong! ABORTING"
  exit 1
  fi

  echo    --- No lastBuild JSON available yet - sleeping for 5 second
  sleep 5
  counter=$((counter+1))
done

counter=0
echo "===== lastBuild info is available ====="
echo "Pinging for finishing of Valgrind report generation"
until [ "`curl --silent --show-error --connect-timeout 1 ${JENKINS_SERVER}/job/generate-static-html/lastBuild/api/json | jq -r '.building'`" == "false" ];
do
  if [ $counter -gt $MAX_RETRIES ]
  then
  echo "Took too long for build to finish -- assuming something went wrong! ABORTING"
  exit 1
  fi

  echo    --- Not finished building yet - sleeping for 20 seconds
  sleep 20
  counter=$((counter+1))
done


echo "===== lastBuild has finished ====="

# result can be SUCCESS | FAILURE
echo "Dumping the console output of the Jenkins job into a file for safe storage!"
curl --silent --connect-timeout 1 ${JENKINS_SERVER}/job/generate-static-html/lastBuild/consoleText > ${OUTPUT_FOLDER}/generate_static_html_consoleText.log
if [ "`curl --silent --show-error --connect-timeout 1 ${JENKINS_SERVER}/job/generate-static-html/lastBuild/api/json | jq -r '.result'`" == "SUCCESS" ]
then
echo "Static html generation successful! Downloading the artifact for publishing in Gitlab!"
else
echo "Static html generaton was a failure! Check the console text log for details."
exit 1
fi

wget -nv -L ${JENKINS_SERVER}/job/generate-static-html/lastSuccessfulBuild/artifact/*zip*/html.zip
mkdir unzip_html
unzip html.zip -d unzip_html
mv unzip_html/archive/static_html/ ${OUTPUT_FOLDER}
rm -r unzip_html/
echo "========= FINISHED THE SCRIPT ========="
