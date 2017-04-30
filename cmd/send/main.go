package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"syscall"

	"github.com/rck/rcswitch"

	"periph.io/x/periph/conn/gpio/gpioreg"
	"periph.io/x/periph/host"
)

const rcPin = 17

func main() {
	flag.Parse()
	args := flag.Args()

	if flag.NArg() != 3 {
		fmt.Fprintln(os.Stderr, "Test program for Type A rc switches")
		fmt.Fprintln(os.Stderr, "Synopsis: send group device state")
		fmt.Fprintln(os.Stderr, "Example: send 11011 10000 1")
		os.Exit(1)
	}

	var status bool
	if args[2] == "1" {
		status = true
	}

	if _, err := host.Init(); err != nil {
		log.Fatal(err)
	}

	pin := gpioreg.ByNumber(rcPin)
	rc := rcswitch.NewRCSwitch(pin)
	syscall.Setpriority(syscall.PRIO_PROCESS, 0, -20)
	if status {
		if err := rc.SwitchOn("", args[0], args[1]); err != nil {
			log.Fatal(err)
		}
	} else {
		if err := rc.SwitchOff("", args[0], args[1]); err != nil {
			log.Fatal(err)
		}
	}
}
