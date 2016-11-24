#SLEEP=2 # time to sleep between samples

#while true; do # or launch it as a cron task
    #TIMESTAMP=$(date +'%s')
    IW=$(iwconfig wlan0)

    # capture each parameter in a variable
    #IEEE=$(echo "$IW" | grep -oP '(?<=IEEE ).[^\s]*')
    #ESSID=$(echo "$IW" | grep -oP '(?<=ESSID:).[^\s]*')
    #BITRATE=$(echo "$IW" | grep -oP '(?<=Bit Rate=)\d+\s.[^\s]+(?=[\s]+Tx)')
    #TXPOWER=$(echo "$IW" | grep -oP '(?<=Tx-Power=)\d+\s.*')
    #RLL=$(echo "$IW" | grep -oP '(?<=limit:)\d')
    #RTSTHR=$(echo "$IW" | grep -oP '(?<=RTS thr:).+(?=[ ]+Fra)')
    #FRAGTHR=$(echo "$IW" | grep -oP '(?<=Fragment thr:).+')
    #TXPOWER=$(echo "$IW" | grep -oP '(?<=Tx-Power=)\d+ .*')
    #PM=$(echo "$IW" | grep -oP '(?<=Management:).*')
    LQ=$(echo "$IW" | grep -oP '(?<=Quality=)\d+/\d+')
    SIGLEV=$(echo "$IW" | grep -oP '(?<=level=).*')
    #RXNWID=$(echo "$IW" | grep -oP '(?<=nwid:)\d+')
    #RXCRYPT=$(echo "$IW" | grep -oP '(?<=crypt:)\d+')
    #RXFRAG=$(echo "$IW" | grep -oP '(?<=frag:)\d+')
    #RTERET=$(echo "$IW" | grep -oP '(?<=retries:)\d+')
    #RTIMISC=$(echo "$IW" | grep -oP '(?<=misc:)\d+')
    #RTMBEAC=$(echo "$IW" | grep -oP '(?<=beacon:)\d+')

    #line
    echo $LQ+$SIGLEV
    #sleep $SLEEP
#done
