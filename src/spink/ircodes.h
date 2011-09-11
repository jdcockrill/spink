/*
  ircodes.h - Infra-red signals that may be
  used to trigger a Canon EOS digital SLR.
  Tested working against my Canon EOS 550D
  Created by jdcockrill, September 11, 2011.
*/
int Canon2SecIRsignal[] = {
// ON, OFF (in 10's of microseconds)
	480, 5200,
	480, 0};

int CanonIRsignal[] = {
// ON, OFF (in 10's of microseconds)
	480, 7100,
	480, 0};

