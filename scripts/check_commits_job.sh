cd wsp/src

echo -e "\e[39mChecking commit status of container repo: tod"

hashHEADtof=$(git rev-parse HEAD)
hashdevelop2tof=$(git rev-parse --verify origin/develop2)
hashmastertof=$(git rev-parse --verify origin/master)

echo "Commit Hash of     HEAD: $hashHEADtof"
echo "Commit Hash of develop2: $hashdevelop2tof"
echo "Commit Hash of   master: $hashmastertof"

checkForDevelop2=false
checkForMaster=false
if [[ "$hashHEADtof" == "$hashdevelop2tof" ]]
then
    echo "HEAD of tof is on develop2, checking each submodule if it is on develop2"
    echo ""
    checkForDevelop2=true
elif [[ "$hashHEADtof" == "$hashmastertof" ]]
then
    echo "HEAD of tof is on master, checking each submodule if it is on master"
    echo ""
    checkForMaster=true
else
    echo "HEAD of tof is neither on develop2 nor on master, setting to check if each submodule is on develop2"
    echo ""
    checkForDevelop2=true
fi


ret=0
for i in tod_*
do
    cd $i
    
    echo -e "\e[39mChecking commit status of package: $i"
    
    hashHEAD=$(git rev-parse HEAD)
    hashdevelop2=$(git rev-parse --verify origin/develop2)
    hashmaster=$(git rev-parse --verify origin/master)
    
    echo "Commit Hash of     HEAD: $hashHEAD"
    echo "Commit Hash of develop2: $hashdevelop2"
    echo "Commit Hash of   master: $hashmaster"

    if [[ "$checkForDevelop2" == "true" ]]
    then
        if [[ "$hashHEAD" == "$hashdevelop2" ]]
        then
            # PASS
            echo -e "\e[92mHEAD of $i is on desired develop2"
        else
            # FAIL
            echo -e "\e[31mHEAD of $i is not on desired develop2"
            ret=1
        fi
    elif [[ "$checkForMaster" == "true" ]]
    then
        if [[ "$hashHEAD" == "$hashmaster" ]]
        then
            # PASS
            echo -e "\e[92mHEAD of $i is on desired master"
        else
            # FAIL
            echo -e "\e[31mHEAD of $i is not on desired master"
            ret=1
        fi
    fi
    echo ""

    cd ..
done
cd ../..

echo -e "\n\e[39mExitting on $ret"
exit $ret
