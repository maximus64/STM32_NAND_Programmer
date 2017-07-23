#!/bin/env python2

import time, sys, serial, os, struct

CMD_READID = 'i'
CMD_READPAGE = 'p'
CMD_READSPARE = 's'
CMD_PING1 = '1'
CMD_PING2 = '2'
CMD_ERASE = 'e'
CMD_WRITEPAGE = 'w'
CMD_WRITESPARE = 'r'
CMD_WRITE_PAGE_SPARE = 'o'

CMD_RET_OK = 0x00
CMD_RET_ERROR = 0xFF

ser = serial.Serial('COM4', 115200)

print "ping FLASHER"
ser.write(CMD_PING1)
ser.flush()
if ser.read() != 'a':
	print "Ping1 Failed"
	exit()
ser.write(CMD_PING2)
ser.flush()
if ser.read() != 'b':
	print "Ping2 Failed"
	exit()
print "ping OK"

def getResult():
	header = ser.read(12)
	result = struct.unpack('<III', header)
	#print "ret: %d sz: %d crc 0x%02x" % (result[0],result[1],result[2])
	return {'ret': result[0], 'sz': result[1], 'crc': result[2]}

def readID():
	ser.write(CMD_READID)
	ser.flush()
	result = getResult()
	if result['ret'] == CMD_RET_OK:
		nandID = ser.read(result['sz'])
		print "NAND ID: ",
		for a in nandID:
			print "%02x" % ord(a),
		print ""
	else:
		print "Error reading NAND ID"


def readspare(page, block, plane):
	ser.write(CMD_READSPARE)
	address = struct.pack('<HHH',page, block, plane)
	ser.write(address)
	ser.flush()
	result = getResult()
	if result['ret'] == CMD_RET_OK:
		spare = ser.read(result['sz'])
		return spare
	else:
		print "Error reading NAND page"
		return None

def readpage(page, block, plane, oob = False):
	ser.write(CMD_READPAGE)
	address = struct.pack('<HHH',page, block, plane)
	ser.write(address)
	ser.flush()
	result = getResult()
	if result['ret'] == CMD_RET_OK:
		data = ser.read(result['sz'])
		if oob:
			data += readspare(page,block,plane)
		return data
	else:
		print "Error reading NAND page"
		return None


def eraseblock(block, plane):
	ser.write(CMD_ERASE)
	address = struct.pack('<HH', block, plane)
	ser.write(address)
	ser.flush()

	result = getResult()
	if result['ret'] == CMD_RET_OK:
		return True
	else:
		print "Erase Error: Fail"
		return False


def programpageoob(page, block, plane, data):
	ser.write(CMD_WRITE_PAGE_SPARE)
	address = struct.pack('<HHH',page, block, plane)
	ser.write(address)
	ser.write(data)
	ser.flush()
	result = getResult()
	if result['ret'] == CMD_RET_OK:
		return True
	else:
		print "Write Error: Fail"
		return False

def programpage(page, block, plane, data):
	ser.write(CMD_WRITEPAGE)
	address = struct.pack('<HHH',page, block, plane)
	ser.write(address)
	ser.write(data)
	ser.flush()
	result = getResult()
	if result['ret'] == CMD_RET_OK:
		return True
	else:
		print "Write Error: Fail"
		return False

def programspare(page, block, plane, data):
	ser.write(CMD_WRITESPARE)
	address = struct.pack('<HHH',page, block, plane)
	ser.write(address)
	ser.write(data)
	ser.flush()
	result = getResult()
	if result['ret'] == CMD_RET_OK:
		return True
	else:
		print "Write Error: Fail"
		return False
#######################################################################


readID()

if len(sys.argv) == 5 and sys.argv[1] == 'ERASEBLOCK':
	plane = int(sys.argv[2],0)
	blk = int(sys.argv[3],0)
	count = int(sys.argv[4],0)

	for block in range(blk,blk+count):
		print "Erasing block: %04x plane: %04x" % (block, plane)
		val = eraseblock(block, plane)
		if val == False:
			print "Erase Failed"
			exit()
		
if len(sys.argv) == 6 and sys.argv[1] == 'WRITEBLOCK':
	plane = int(sys.argv[2],0)
	blk = int(sys.argv[3],0)
	count = int(sys.argv[4],0)

	infile = open(sys.argv[5],"rb")
	infile.seek(0,os.SEEK_END)
	filesize = infile.tell()

	if filesize != (4096+224) * 256 * count:
		print "File size not correct"
		#exit()

	infile.seek(0,os.SEEK_SET)

	print "Write start at block: 0x%04x" % blk

	for block in range(blk,blk+count):
		for page in range(0, 0x100):
			print "Writing page: %04x block: %04x plane: %04x" % (page, block, plane)
			# data = infile.read(4096)
			# val = programpage(page, block, plane, data)
			# if val == False:
			# 	print "Write Failed"
			# 	exit()
			# data = infile.read(224)
			# val = programspare(page, block, plane, data)
			# if val == False:
			# 	print "Write Failed"
			# 	exit()

			data = infile.read(4096+224)
			val = programpageoob(page, block, plane, data)
			if val == False:
				print "Write Failed"
				exit()
		
	infile.close()

if len(sys.argv) == 3 and sys.argv[1] == 'TEST':
	count = int(sys.argv[2])
	for currpos in range(0, count):
		print "Reading page: %05x" % currpos
		data = readpage(currpos,0,0)
		data2 = readpage(currpos,0,0)
		if data != data2:
			print "FAILED"
			break


if len(sys.argv) == 3 and sys.argv[1] == 'DUMPNAND':
	outfile = open(sys.argv[2],"wb")
	for plane in range(0, 0x2):
		for block in range(0, 0x400):
			for page in range(0, 0x100):
				print "Reading page: %04x block: %04x plane: %04x" % (page, block, plane)
				data = readpage(page, block, plane, True)
				outfile.write(data)
				outfile.flush()
	outfile.close()

if len(sys.argv) == 6 and sys.argv[1] == 'DUMPBLOCK':
	outfile = open(sys.argv[5],"wb")
	plane = int(sys.argv[2],0)
	blk = int(sys.argv[3],0)
	count = int(sys.argv[4],0)

	for block in range(blk, blk+count):
		for page in range(0, 0x100):
			print "Reading page: %04x block: %04x plane: %04x" % (page, block, plane)
			data = readpage(page, block, plane, True)
			outfile.write(data)
			outfile.flush()

	outfile.close()
		

ser.close()
