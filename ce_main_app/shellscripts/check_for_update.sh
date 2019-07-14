#!/bin/sh

echo " "
echo "Checking for update..."

# delete both update_pending answer files
rm -f /tmp/UPDATE_PENDING_YES
rm -f /tmp/UPDATE_PENDING_NO

read_from_file()
{
    if [ -f $1 ]; then      # does the file exist? read value from file
        val=$( cat $1 )
    else                    # file doesn't exist? use default value
        val=$2
    fi

    echo $val               # display the value
}

distro=$( /ce/whichdistro.sh )

url_to_git_api="https://api.github.com/repos/atarijookie/ce-atari-releases/commits/$distro"

echo "We're running on distro: $distro"

# on yocto - not actually checking, just do the update
if [ "$distro" = "yocto" ]; then
    echo "Manually checking for last commit, please wait..."

    # get last online commit in the branch for this distro
    last_online_commit=$( wget -O- $url_to_git_api 2> /dev/null | grep -m 1 'sha' | sed 's/sha//g' | sed 's/[\" \t\n:,]//g' )

    # try to read currently stored (used) commit from file
    commit_curr=$( read_from_file /ce/update/commit.curent 0 )

    if [ "$commit_curr" != "$last_online_commit" ]; then    # commits are different? do update
        echo "Found new commit online, will update"
        echo "( $commit_curr != $last_online_commit )"
        touch /tmp/UPDATE_PENDING_YES
    else        # commits are the same? don't update
        echo "No new commit online, won't update"
        touch /tmp/UPDATE_PENDING_NO
    fi

    exit
fi

# check if got git installed
git --version 2> /dev/null

if [ "$?" -ne "0" ]; then           # if don't have git, then run the update, which will try to install git
    echo "don't have git, will update"
    touch /tmp/UPDATE_PENDING_YES
    exit
fi

echo "checking for new files"

cd /ce/                             # go to /ce directory
git remote update 2> /dev/null      # try git remote update

if [ "$?" -ne "0" ]; then           # git complained that this is not a repo? run the update
    echo "/ce/ is not a git repo yet, will update"
    touch /tmp/UPDATE_PENDING_YES
    exit
fi

# check if we're behind repo or just fine
output=$( git status -uno | grep 'behind' | wc -l )

if [ "$output" -gt "0" ]; then      # if 'behind' was found in the status
    echo "we're behind remote repo, do update"
    touch /tmp/UPDATE_PENDING_YES
else                                # not behind, no update pending
    echo "no changes found, don't do update"
    touch /tmp/UPDATE_PENDING_NO
fi
