# for Proficio
alias leftConfig='cp ~/libbarrett/proficio_sandbox/configurations/wam3.conf.left /etc/barrett/wam3.conf; cp ~/libbarrett/proficio_sandbox/configurations/zerocal.conf.left /etc/barrett/calibration_data/wam3/zerocal.conf; cp ~/libbarrett/proficio_sandbox/configurations/calibration.conf.left /etc/barrett/calibration.conf; cp ~/libbarrett/proficio_sandbox/configurations/gravitycal.conf.left /etc/barrett/calibration_data/wam3/gravitycal.conf'
alias rightConfig='cp ~/libbarrett/proficio_sandbox/configurations/wam3.conf.right /etc/barrett/wam3.conf; cp ~/libbarrett/proficio_sandbox/configurations/zerocal.conf.right /etc/barrett/calibration_data/wam3/zerocal.conf; cp ~/libbarrett/proficio_sandbox/configurations/calibration.conf.right /etc/barrett/calibration.conf'
alias checkSetup='head -n 1 /etc/barrett/wam3.conf; head -n 1 /etc/barrett/calibration.conf; head -n 1 /etc/barrett/calibration_data/wam3/zerocal.conf'
alias btutil='~/btclient/src/btutil/btutil'
alias p3util='~/btclient/src/p3util/p3util'
alias resetcan='sudo sh ~/btclient/reset_can.sh'
