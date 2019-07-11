#!/bin/sh
BRANCH=$(git rev-parse --abbrev-ref HEAD)
DESCRIBE=$(git describe --always --dirty)
DATE=$(date +"%Y-%m-%d")
FULLDATE=$(date)
SAFE_BRANCH=$(echo $TRAVIS_BRANCH | sed 's/[^a-zA-Z0-9\._\-\:\+]/_/g' -)
export DESCRIBE DATE BRANCH FULLDATE
envsubst <<EOF
{
    "package" :{
        "name": "$SAFE_BRANCH",
        "description": "Automated build results of the $TRAVIS_BRANCH branch",
        "repo": "TSDZ2-Clean-EBike",
        "subject": "frans-willem",
        "vcs_url": "https://github.com/Frans-Willem/TSDZ2-Clean-EBike.git",
        "licenses": ["GPL-3.0"]
    },
    "version": {
        "name": "build$TRAVIS_BUILD_NUMBER-$DESCRIBE",
        "desc": "This is a build of commit $DESCRIBE on branch $TRAVIS_BRANCH built on $FULLDATE",
        "released": "$DATE",
        "vcs_tag": "$DESCRIBE",
        "attributes": [],
        "gpgSign": false
    },
    "files":
        [
           {"includePattern": "package\.zip", "uploadPattern": "TSDZ2-Clean-Ebike-$SAFE_BRANCH-build$TRAVIS_BUILD_NUMBER-$DESCRIBE.zip"}
        ],
    "publish": true
}
EOF
