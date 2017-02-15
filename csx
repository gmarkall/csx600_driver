# Copyright (c) 1995-2004 SUSE Linux AG, Nuernberg, Germany.
# All rights reserved.
#
# Author: Kurt Garloff
# Please send feedback to http://www.suse.de/feedback/
#
# /etc/init.d/csx
#   and its symbolic link
# /(usr/)sbin/rccsx600
#
# System startup script for csx
#
# LSB compatible service control script; see http://www.linuxbase.org/spec/
# 
# Note: This template uses functions rc_XXX defined in /etc/rc.status on
# UnitedLinux (UL) based Linux distributions. If you want to base your 
# script on this template and ensure that it works on non UL based LSB 
# compliant Linux distributions, you either have to provide the rc.status
# functions from UL or change the script to work without them.
#
# chkconfig: 2345 85 15
# description: load GPL'd Clearspeed CSX600 device driver module
### BEGIN INIT INFO
# Provides:          csx
# Required-Start:    $syslog $remote_fs
# Should-Start: 
# Required-Stop:     $syslog $remote_fs
# Should-Stop:
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: GPL'd Clearspeed CSX600 device driver module
# Description:       load GPL'd Clearspeed CSX600 device driver module
#	(The Short-Description should already be a good hint.)
### END INIT INFO
# 
# Any extensions to the keywords given above should be preceeded by 
# X-VendorTag- (X-UnitedLinux- X-SuSE- for us) according to LSB.
# 
# Notes on Required-Start/Should-Start:
# * There are two different issues that are solved by Required-Start
#    and Should-Start
# (a) Hard dependencies: This is used by the runlevel editor to determine
#     which services absolutely need to be started to make the start of
#     this service make sense. Example: nfsserver should have
#     Required-Start: $portmap
#     Also, required services are started before the dependent ones.
#     The runlevel editor will warn about such missing hard dependencies
#     and suggest enabling. During system startup, you may expect an error,
#     if the dependency is not fulfilled.
# (b) Specifying the init script ordering, not real (hard) dependencies.
#     This is needed by insserv to determine which service should be
#     started first (and at a later stage what services can be started
#     in parallel). The tag Should-Start: is used for this.
#     It tells, that if a service is available, it should be started
#     before. If not, never mind.
# * When specifying hard dependencies or ordering requirements, you can 
#   use names of services (contents of their Provides: section)
#   or pseudo names starting with a $. The following ones are available
#   according to LSB (1.1):
#	$local_fs		all local file systems are mounted
#				(most services should need this!)
#	$remote_fs		all remote file systems are mounted
#				(note that /usr may be remote, so
#				 many services should Require this!)
#	$syslog			system logging facility up
#	$network		low level networking (eth card, ...)
#	$named			hostname resolution available
#	$netdaemons		all network daemons are running
#   The $netdaemons pseudo service has been removed in LSB 1.2.
#   For now, we still offer it for backward compatibility.
#   These are new (LSB 1.2):
#	$time			the system time has been set correctly	
#	$portmap		SunRPC portmapping service available
#   UnitedLinux extensions:
#	$ALL			indicates that a script should be inserted
#				at the end
# * The services specified in the stop tags 
#   (Required-Stop/Should-Stop)
#   specify which services need to be still running when this service
#   is shut down. Often the entries there are just copies or a subset 
#   from the respective start tag.
# * Should-Start/Stop are now part of LSB as of 2.0,
#   formerly SUSE/Unitedlinux used X-UnitedLinux-Should-Start/-Stop.
#   insserv does support both variants.
# * X-UnitedLinux-Default-Enabled: yes/no is used at installation time
#   (%fillup_and_insserv macro in %post of many RPMs) to specify whether
#   a startup script should default to be enabled after installation.
#   It's not used by insserv.
#
# Note on runlevels:
# 0 - halt/poweroff 			6 - reboot
# 1 - single user			2 - multiuser without network exported
# 3 - multiuser w/ network (text mode)  5 - multiuser w/ network and X11 (xdm)
# 
# Note on script names:
# http://www.linuxbase.org/spec/refspecs/LSB_1.3.0/gLSB/gLSB/scrptnames.html
# A registry has been set up to manage the init script namespace.
# http://www.lanana.org/
# Please use the names already registered or register one or use a
# vendor prefix.


# Source LSB init functions
# providing start_daemon, killproc, pidofproc, 
# log_success_msg, log_failure_msg and log_warning_msg.
# This is currently not used by UnitedLinux based distributions and
# not needed for init scripts for UnitedLinux only. If it is used,
# the functions from rc.status should not be sourced or used.
if [ -f /lib/lsb/init-functions ]; then
  . /lib/lsb/init-functions
  LOG_SUCCESS(){ RC=$?; eval log_success_msg $*; return $RC; }
  LOG_FAILURE(){ RC=$?; eval log_failure_msg $*; return $RC; }
  LOG_WARNING(){ RC=$?; eval log_warning_msg $*; return $RC; }
#  echo LSB
elif [ -f /etc/init.d/functions ]; then
  . /etc/init.d/functions
  alias LOG_SUCCESS=success
  alias LOG_FAILURE=failure
  alias LOG_WARNING=warning
#  echo REDHAT
else
  echo "Error: your platform is not supported by $0" > /dev/stderr
  exit 1
fi

# Shell functions sourced from /etc/rc.status:
#      rc_check         check and set local and overall rc status
#      rc_status        check and set local and overall rc status
#      rc_status -v     be verbose in local rc status and clear it afterwards
#      rc_status -v -r  ditto and clear both the local and overall rc status
#      rc_status -s     display "skipped" and exit with status 3
#      rc_status -u     display "unused" and exit with status 3
#      rc_failed        set local and overall rc status to failed
#      rc_failed <num>  set local and overall rc status to <num>
#      rc_reset         clear both the local and overall rc status
#      rc_exit          exit appropriate to overall rc status
#      rc_active        checks whether a service is activated by symlinks
#      rc_splash arg    sets the boot splash screen to arg (if active)
#. /etc/rc.status

# Reset status of this service
#rc_reset

# Return values acc. to LSB for all commands but status:
# 0	  - success
# 1       - generic or unspecified error
# 2       - invalid or excess argument(s)
# 3       - unimplemented feature (e.g. "reload")
# 4       - user had insufficient privileges
# 5       - program is not installed
# 6       - program is not configured
# 7       - program is not running
# 8--199  - reserved (8--99 LSB, 100--149 distrib, 150--199 appl)
# 
# Note that starting an already running service, stopping
# or restarting a not-running service as well as the restart
# with force-reload (in case signaling is not supported) are
# considered a success.
myname=`basename $0`

case "$1" in
    start)

	echo -n "Starting csx driver "
    # Plan:
    # start the driver as normal, this should allow the csx driver and udev to
    #  create the correct sys structure and the corresponding /dev entries
    #  wait a while for everything to settle
    # then, do a 'lspci -d', wc, board count check
    #  make sure everything exists in /dev/csx* accordingly
    # if it doesnt try to create the entries based on which version
    #  of udev we have available
    # correct method is to check for /sbin/udev and get it to fire up accordingly
    #  if this is not available we have no choice but to call mknod directly

	## Start daemon with startproc(8). If this fails
	## the return value is set appropriately by startproc.
	/sbin/modprobe csx
    status=$?

    if [ "$status" = 0 ]
    then
      # sleep 4 seconds to allow device creation
      sleep 4

      # Now get the number of boards we think are present in the system
      # and the number of /dev/csx*c/m entries present
      boards_counted=`/bin/lspci -d 0x1942:|wc -l`
      ctl_entries_counted=`/bin/ls -al /dev/csx*c 2>/dev/null|wc -l`
      mem_entries_counted=`/bin/ls -al /dev/csx*m 2>/dev/null|wc -l`

      # Now make sure the numbers all match up for the ctl entries
      if [ $boards_counted -ne $ctl_entries_counted -o $boards_counted -ne $mem_entries_counted ]
      then
        # We rectify this by going through the /sys/class/csxtl/csx directory
        #  and making sure everything present is created properly
        curr_dir=`pwd`
        cd /sys
        ctl_dev=`/bin/ls -d class/csxctl/csx* 2>/dev/null`
        mem_dev=`/bin/ls -d class/csxmem/csx* 2>/dev/null`
        cd $curr_dir

        for filename in $ctl_dev;
        do
          # Only do this for entries that DONT exist
          dev_name=`echo $filename|cut -f 3 -d '/'`
          if [ ! -c $dev_name ]
          then
            if [ -x /sbin/udev ]
            then
              ACTION=add DEVPATH=/$filename /sbin/udev csxctl;
            else
              # No other option but to call mknod directly
              major=`/bin/cat /sys/$filename/dev|cut -f 1 -d ':'`
              minor=`/bin/cat /sys/$filename/dev|cut -f 2 -d ':'`
              name=`echo $filename|cut -f 3 -d '/'`
              /bin/mknod /dev/$name c $major $minor
            fi
          fi
        done

        for filename in $mem_dev;
        do
          # Only do this for entries that DONT exist
          dev_name=`echo $filename|cut -f 3 -d '/'`
          if [ ! -c $dev_name ]
          then
            if [ -x /sbin/udev ]
            then
              ACTION=add DEVPATH=/$filename /sbin/udev csxmem;
            else
              # No other option but to call mknod directly
              major=`/bin/cat /sys/$filename/dev|cut -f 1 -d ':'`
              minor=`/bin/cat /sys/$filename/dev|cut -f 2 -d ':'`
              name=`echo $filename|cut -f 3 -d '/'`
              /bin/mknod /dev/$name c $major $minor
            fi
          fi
        done
      fi
    fi

	# Remember status and be verbose
	#rc_status -v
    if [ "$status" = 0 ]
      then LOG_SUCCESS $myname
      else LOG_FAILURE $myname
    fi

    # Go and make sure the permissions are all okay
    /bin/chmod a+rw /dev/csx[0-9]{m,c} /dev/csx[0-9][0-9]{m,c} 2>/dev/null
    ;;

    stop)
        # Check to make sure it is safe for us to stop the driver module
        # ( we do this by looking at the /var/.../cs_lock_file.txt )
        lock_file_lines=`/bin/cat /var/lock/clearspeed/cs_lock_file.txt 2>/dev/null|grep -v '#'|wc|sed -e 's/  */ /g'|cut -f 2 -d ' '`
        process_active=0

        if [ $lock_file_lines -gt 0 ]
        then
          # Now check if the specified process id's are still active
          pid_list=`/bin/cat /var/lock/clearspeed/cs_lock_file.txt|grep -v '#'|sed -n '5~7p'`

          for pid in $pid_list; do
            active=`/bin/ps -deaf|grep $pid|grep -v grep|wc -l`
            if [ $active -gt 0 ]
            then
              process_active=1
              echo "Process "$pid" is active for user "`ps -deaf|grep $pid|grep -v grep|cut -f 1 -d ' '`
            fi
          done
        fi
  
        if [ $process_active -gt 0 ]
        then
          echo "ERROR: csx driver is currently ACTIVE"
          echo "Lock file reports:"
          /bin/cat /var/lock/clearspeed/cs_lock_file.txt|grep "Locked by"|sed 's/^/     /'
          status=1
        else
#          echo "No lock present, okay to remove driver"
          echo -n "Shutting down csx driver "
          /sbin/modprobe -r csx
          status=$?

          # udev doesn't always delete the /dev entry either
          # especially on RHE4_64 and SLES9.3
          if [ "$status" = 0 ] ; then
            rm -f  /dev/csx[0-9]{m,c} /dev/csx[0-9][0-9]{m,c}
          fi

          # Remember status and be verbose
          #rc_status -v
          if [ "$status" = 0 ]
          then LOG_SUCCESS $myname
          else LOG_FAILURE $myname
          fi
        fi
	;;
    try-restart|condrestart)
	## Do a restart only if the service was active before.
	## Note: try-restart is now part of LSB (as of 1.9).
	## RH has a similar command named condrestart.
	if test "$1" = "condrestart"; then
		echo "${attn} Use try-restart ${done}(LSB)${attn} rather than condrestart ${warn}(RH)${norm}"
	fi
	$0 status
	if test $? = 0; then
		$0 restart
	else
		#rc_reset	# Not running is not a failure.
        :
	fi
	# Remember status and be quiet
	#rc_status
    #if [ "$?" = 0 ]
    #then LOG_SUCCESS $myname
    #else LOG_FAILURE $myname
    #fi
	;;
    restart)
	## Stop the service and regardless of whether it was
	## running or not, start it again.
	$0 stop
	$0 start

	# Remember status and be quiet
	#rc_status
    #if [ "$?" = 0 ]
    #then LOG_SUCCESS $myname
    #else LOG_FAILURE $myname
    #fi
	;;
    force-reload)
	## Signal the daemon to reload its config. Most daemons
	## do this on signal 1 (SIGHUP).
	## If it does not support it, restart.

	echo -n "Reload csx driver "

	## Otherwise:
	$0 try-restart
	#rc_status
    #if [ "$?" = 0 ]
    #then LOG_SUCCESS $myname
    #else LOG_FAILURE $myname
    #fi
	;;
    reload)
	## Like force-reload, but if daemon does not support
	## signaling, do nothing (!)

	## Otherwise if it does not support reload:
	#rc_failed 3
    (exit 3)
	#rc_status -v
    LOG_FAILURE $myname
	;;
    status)
	echo -n "Checking for service csx driver "
	## Check status with checkproc(8), if process is running
	## checkproc will return with exit status 0.

	# Return value is slightly different for the status command:
	# 0 - service up and running
	# 1 - service dead, but /var/run/  pid  file exists
	# 2 - service dead, but /var/lock/ lock file exists
	# 3 - service not running (unused)
	# 4 - service status unknown :-(
	# 5--199 reserved (5--99 LSB, 100--149 distro, 150--199 appl.)
	
	# NOTE: checkproc returns LSB compliant status values.
	/sbin/lsmod | grep "csx " > /dev/null
	# NOTE: rc_status knows that we called this init script with
	# "status" option and adapts its messages accordingly.
	#rc_status -v
    if [ "$?" = 0 ]
    then LOG_SUCCESS $myname
    else LOG_FAILURE $myname
    fi
	;;
    probe)
	## Optional: Probe for the necessity of a reload, print out the
	## argument to this init script which is required for a reload.
	## Note: probe is not (yet) part of LSB (as of 1.9)

	;;
    *)
	echo "Usage: $0 {start|stop|status|try-restart|restart|force-reload|reload|probe}"
	exit 1
	;;
esac
exit $?
