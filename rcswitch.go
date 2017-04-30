// Package rcswitch provides a library to control 433/315MHz devices like power outlet sockets.
// This follows the upstream (https://github.com/sui77/rc-switch) C++ implementation.
package rcswitch

import (
	"errors"
	"fmt"
	"strconv"
	"strings"
	"sync"
	"time"

	"periph.io/x/periph/conn/gpio"
)

type waveform struct {
	high, low int // Number of high pulses, followed by number of low pulses.
}

type protocol struct {
	pulseLen                 time.Duration
	syncBit, zeroBit, oneBit waveform
	inverted                 bool
}

var protocols = []protocol{
	// protocol 1
	{pulseLen: 350, syncBit: waveform{1, 31}, zeroBit: waveform{1, 3}, oneBit: waveform{3, 1}},
	// protocol 2
	{pulseLen: 650, syncBit: waveform{1, 10}, zeroBit: waveform{1, 2}, oneBit: waveform{2, 1}},
	// protocol 3
	{pulseLen: 100, syncBit: waveform{30, 71}, zeroBit: waveform{4, 11}, oneBit: waveform{9, 6}},
	// protocol 4
	{pulseLen: 380, syncBit: waveform{1, 6}, zeroBit: waveform{1, 3}, oneBit: waveform{3, 1}},
	// protocol 5
	{pulseLen: 500, syncBit: waveform{6, 14}, zeroBit: waveform{1, 2}, oneBit: waveform{2, 1}},
	// protocol 6 (HT6P20B)
	{pulseLen: 450, syncBit: waveform{23, 1}, zeroBit: waveform{1, 2}, oneBit: waveform{2, 1}, inverted: true},
}

// The RCSwitch object.
type RCSwitch struct {
	pin      gpio.PinIO
	protocol protocol
	nrRepeat int
	isOn     map[string]bool
	sync.Mutex
}

// Create RCSwitch object for the given pin.
func NewRCSwitch(pin gpio.PinIO) *RCSwitch {
	s := RCSwitch{
		nrRepeat: 10,
	}

	s.isOn = make(map[string]bool)
	s.SetPin(pin)
	s.SetProtocol(1)
	return &s
}

// Set the pin of the RCSwitch object.
func (s *RCSwitch) SetPin(pin gpio.PinIO) {
	s.Lock()
	s.pin = pin
	s.Unlock()
}

// A wave form (e.g., for "on") is sent this number of times.
// The default is 10.
func (s *RCSwitch) SetRepeat(nrRepeat int) error {
	if nrRepeat <= 0 {
		return errors.New("Repeat has to be a positive number")
	}
	s.Lock()
	s.nrRepeat = nrRepeat
	s.Unlock()
	return nil
}

// Set the protocol used for transmission.
// The default is the most common protocol 1.
func (s *RCSwitch) SetProtocol(protocol int) error {
	if protocol <= 0 || protocol > len(protocols) {
		return fmt.Errorf("Protocol %d is not supported, supported are 1 to %d", protocol, len(protocols))
	}
	s.Lock()
	s.protocol = protocols[protocol-1]
	s.Unlock()
	return nil
}

// Turn on a switch.
// Group and device have to be set.
// Family is only used for Type C. In the most common case family is unused and should be set to "".
// Type A (most common): family: "", group: binary string (e.g. "11011"), device: binary string (e.g, "10000").
// Type B: family: "", group: string 1-4 (e.g. "1"), device: string 1-4 (e.g, "2").
// Type C: family: string a-f (e.g. "b"), group: string 1-4 (e.g. "1"), device: string 1-4 (e.g, "2").
// Type D: family: "", group: string a-d (e.g. "a"), device: string 1-3 (e.g, "2").
func (s *RCSwitch) SwitchOn(family, group, device string) error {
	s.Lock()
	defer s.Unlock()
	code, err := getCodeWord(family, group, device, true)
	if err != nil {
		return err
	}
	s.sendTriState(code)
	// changing the codeword type between different calls to On/Off does not make sense, so group+device is unique
	s.isOn[group+device] = true
	return nil
}

// Turn on a switch. Format is the same as for SwitchOn.
func (s *RCSwitch) SwitchOff(family, group, device string) error {
	s.Lock()
	defer s.Unlock()
	code, err := getCodeWord(family, group, device, false)
	if err != nil {
		return err
	}
	s.sendTriState(code)
	s.isOn[group+device] = false
	return nil
}

// Returns true if the switch is "on".
// This is just a state kept within the RCSwitch object. It does not reflect
// the physical state. For example if the switch was manually turned on, it is
// reported as "off" for the first call. Usually you want to call SwitchOff to
// get this into a known state where the physical state matches the one tracked
// in the object. Still, if for example sending "off" does not actually switch
// the switch off (e.g., interference in the transmission), it will be tracked
// as off even if it physically is still on.
func (s *RCSwitch) IsOn(group, device string) bool {
	s.Lock()
	defer s.Unlock()
	return s.isOn[group+device]
}

func (s *RCSwitch) sendTriState(tristate string) {
	s.send(triStateToBinary(tristate))
}

func (s *RCSwitch) send(binary string) {
	ws := binaryToWaveForm(binary, s.protocol)
	transmit(&ws, s.protocol, s.nrRepeat, s.pin)
}

// The C++ implementation was called for every single waveform.
// Handing over the whole slice without calling the function multiple times
// (250 times is not uncommon with the default repeat factor) makes timing more
// reliable. This was an issue on my old, first gen raspi.
func transmit(ws *[]waveform, prot protocol, nrRepeat int, pin gpio.PinIO) {
	d := prot.pulseLen * time.Microsecond

	f, s := gpio.High, gpio.Low
	if prot.inverted {
		f, s = s, f
	}

	for i := 0; i < nrRepeat; i++ {
		for _, w := range *ws {
			pin.Out(f)
			time.Sleep(time.Duration(w.high) * d)
			pin.Out(s)
			time.Sleep(time.Duration(w.low) * d)
		}
	}
}

func getCodeWord(family, group, device string, status bool) (string, error) {
	if family != "" { // Type C
		return getCodeWordC(family, group, device, status)
	}

	if len(group) > 1 && len(device) > 1 { // Type A
		return getCodeWordA(group, device, status)
	}

	if len(group) == 1 && len(device) == 1 { // Type B or D
		// both have an integer device
		d, err := strconv.Atoi(device)
		if err != nil {
			return "", errors.New("Protocols B/D have a device string that can be converted to an integer")
		}
		g, err := strconv.Atoi(group)
		if err != nil { // Type B
			return getCodeWordB(g, d, status)
		} else { // Type D
			return getCodeWordD(group, d, status)
		}
	}

	return "", errors.New("family, group, device combination not supported")
}

func getCodeWordA(group, device string, status bool) (string, error) {
	if len(group) != 5 {
		return "", errors.New("Group has to have a length of 5 encoded as binary (e.g., 11011)")
	}
	if len(device) != 5 {
		return "", errors.New("Device has to have a length of 5 encoded as binary (e.g., 10000)")
	}

	var codeword string

	for _, b := range group + device {
		if b == '0' {
			codeword += "F"
		} else {
			codeword += "0"
		}
	}

	if status {
		codeword += "0F"
	} else {
		codeword += "F0"
	}

	return codeword, nil
}

// This is untested, if you can test it, please send a pull request removing this comment and add a test case.
func getCodeWordB(group, device int, status bool) (string, error) {
	if group < 1 || group > 4 || device < 1 || device > 4 {
		return "", errors.New("Group and device have to be within the range of 1 to 4")
	}

	var codeword string
	for i := 1; i <= 4; i++ {
		if group == i {
			codeword += "0"
		} else {
			codeword += "F"
		}
	}

	for i := 1; i <= 4; i++ {
		if device == i {
			codeword += "0"
		} else {
			codeword += "F"
		}
	}

	codeword += "FFF"

	if status {
		codeword += "F"
	} else {
		codeword += "0"
	}

	return codeword, nil
}

// This is untested, if you can test it, please send a pull request removing this comment and add a test case.
func getCodeWordC(family, group, device string, status bool) (string, error) {
	if len(family) != 1 {
		return "", errors.New("Family has to be a single character")
	}

	f, err := strconv.ParseUint(family, 16, 8) // implicetly contains a..f check
	if err != nil {
		return "", err
	}

	g, err := strconv.Atoi(group)
	if err != nil {
		return "", err
	}
	if g < 1 || g > 4 {
		return "", errors.New("Group has to be between 1 and 4")
	}

	d, err := strconv.Atoi(device)
	if err != nil {
		return "", err
	}
	if d < 1 || d > 4 {
		return "", errors.New("Device has to be between 1 and 4")
	}

	var codeword string

	for i := uint(0); i < 4; i++ {
		if (f & 0x1) == 0x1 {
			codeword += "F"
		} else {
			codeword += "0"
		}
		f >>= 1
	}

	conf := func(i int) string {
		var s string
		iu := uint(i) - 1
		if iu&0x1 == 1 {
			s += "F"
		} else {
			s += "0"
		}
		if iu&0x2 == 1 {
			s += "F"
		} else {
			s += "0"
		}
		return s
	}

	codeword += conf(d)
	codeword += conf(g)

	// status
	codeword += "0FF"
	if status {
		codeword += "F"
	} else {
		codeword += "0"
	}

	return codeword, nil
}

// This is untested, if you can test it, please send a pull request removing this comment and add a test case.
func getCodeWordD(group string, device int, status bool) (string, error) {
	if len(group) != 1 {
		return "", errors.New("Group has to be a single character")
	}

	var codeword string

	switch strings.ToLower(group) {
	case "a":
		codeword += "1FFF"
	case "b":
		codeword += "F1FF"
	case "c":
		codeword += "FF1F"
	case "d":
		codeword += "FFF1"
	default:
		return "", errors.New("Group has to be in a-d or A-D")
	}

	//TODO(rck): this matches the implementation, but the upstream description is different, bug got reported upstream
	switch device {
	case 1:
		codeword += "1FF"
	case 2:
		codeword += "F1F"
	case 3:
		codeword += "FF1"
	default:
		return "", errors.New("Group has to be in the range of 1..3")
	}

	// unused
	codeword += "000"

	// status
	if status {
		codeword += "10"
	} else {
		codeword += "01"
	}

	return codeword, nil
}

func triStateToBinary(tristate string) string {
	var binary string
	for _, c := range tristate {
		switch c {
		case '0':
			binary += "00"
		case '1':
			binary += "11"
		case 'F':
			binary += "01"
		}
	}
	return binary
}

func binaryToWaveForm(binary string, prot protocol) []waveform {
	ws := make([]waveform, 0, len(binary)+1)
	for _, b := range binary {
		if b == '1' {
			ws = append(ws, prot.oneBit)
		} else {
			ws = append(ws, prot.zeroBit)
		}
	}
	ws = append(ws, prot.syncBit)
	return ws
}
