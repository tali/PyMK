==========================================
Intel HEX file format reader and convertor
==========================================
---------------------
Python implementation
---------------------

:Author: Alexander Belchenko
:Contact: bialix AT ukr net
:Date: 2007-04-27
:Version: 0.8.6

.. Contents::

Introduction
------------
Intel HEX file format widely used in microprocessors and microcontrollers
area as de-facto standard for representation of code for programming into
microelectronic devices.

This work implements HEX (also known as Intel HEX) file format reader
and convertor to binary form as python script.

Script intelhex.py contain implementation of HEX file reader and convertor
as IntelHex class. You also may use this script as standalone hex-to-bin
convertor.


License
-------
The code distributed under BSD license. See LICENSE.txt in sources achive.


Download
--------
http://www.bialix.com/intelhex/0.8/intelhex-0.8.6.zip


Project at Launchpad
--------------------
Intelhex project at Launchpad.net:

    https://launchpad.net/intelhex/

There is bug tracker and source code history browser. I use Bazaar version
control system for development of intelhex.
    
Bzr (Bazaar version-control system) itself is here: 
http://bazaar-vcs.org


IntelHex classes
----------------
Basic
*****
Example of typical usage of ``IntelHex`` class::

	>>> from intelhex import IntelHex	# 1
	>>> h = IntelHex("foo.hex")		# 2
	>>> h.readfile()			# 3
	>>> h.tobinfile("foo.bin")		# 4

In second line we are create instance of class. Constructor has one parameter:
name of HEX file or file object. For reading and decoding content of HEX file
we need to invoke ``readfile()`` method (see line 3 in example above). 
It returns True if file processed successful, or False if some error detected.
Class IntelHex has 3 methods for converting content of HEX file 
into binary form (see line 4 in example above):

	* ``tobinarray`` (returns array of unsigned char bytes);
	* ``tobinstr``   (returns string of bytes);
	* ``tobinfile``  (convert content to binary form and write to file).

Access to data by address
*************************
You can get or modify some data by address in usual way: via indexing
operations::

	>>> print ih[0]			# read data from address 0
        >>> ih[1] = 0x55		# modify data at address 1

When you try to read from non-existent address you get default data. Default
data sets via attribute ``.padding`` of class instance.

To obtain adresses limits use methods ``.minaddr()`` and ``.maxaddr()``.

Access to 16-bit data
*********************
When you need to work with 16-bit data stored in 8-bit Intel HEX file you need
to use class ``IntelHex16bit``. This class derived from IntelHex and has all their
methods. But some of methods modified to implement 16-bit behaviour.

Write data to HEX file
**********************
You can store data contained in object by method ``.writefile(f)``. Parameter
``f`` should be filename or file-like object.

Start address
*************
Some linkers write to produced HEX file information about start address
(either record 03 or 05). Now IntelHex is able correctly read such records
and store information internally in ``start_addr`` attribute that itself
is ``None`` or dictionary with address value(s). 

When input HEX file contains record type 03 (Start Segment Address Record),
``start_addr`` takes value::

	{'CS': XXX, 'IP': YYY}
        
Here:

	* ``XXX`` is value of CS register
	* ``YYY`` is value of IP register

To obtain or change ``CS`` or ``IP`` value you need to use their names
as keys for ``start_addr`` dictionary::

	>>> ih = IntelHex('file_with_03.hex')
	>>> ih.readfile()
	>>> print ih.start_addr['CS']
	>>> print ih.start_addr['IP']

When input HEX file contains record type 05 (Start Linear Address Record),
``start_addr`` takes value::

	{'EIP': ZZZ}

Here ``ZZZ`` is value of EIP register.

Example::

	>>> ih = IntelHex('file_with_05.hex')
	>>> ih.readfile()
	>>> print ih.start_addr['EIP']

You can manually set required start address::

	>>> ih.start_addr = {'CS': 0x1234, 'IP': 0x5678}
	>>> ih.start_addr = {'EIP': 0x12345678}

To delete start address info give value ``None`` or empty dictionary::

	>>> ih.start_addr = None
	>>> ih.start_addr = {}

When you write data to HEX file you can disable writing start address
with additional argument ``write_start_addr``:

	>>> ih.writefile('out.hex')	# by default writing start address
	>>> ih.writefile('out.hex', True)	# as above
	>>> ih.writefile('out.hex', False)	# don't write start address

When ``start_addr`` is ``None`` or empty dictionary nothing will be written
regardless of ``write_start_addr`` argument value.


Documentation
-------------
You can use epydoc_ for creating documentation for IntelHex class. Run epydoc::

	$ python epydoc.py intelhex.py

.. _epydoc: http://epydoc.sourceforge.net/


Hex-to-Bin convertor
--------------------
You can use hex-to-bin convertor in two way: as function ``hex2bin`` (useful
for using in other scripts) or as stand-alone script.

Function ``hex2bin``
********************
Hex-to-Bin convertor engine.

``hex2bin(fin, fout, start=None, end=None, size=None, pad=255)``

**Parameters**: 

* ``fin`` -- input hex file (filename or file-like object) 
* ``fout`` -- output bin file (filename or file-like object) 
* ``start`` -- start of address range (optional) 
* ``end`` -- end of address range (optional) 
* ``size`` -- size of resulting file (in bytes) (optional) 
* ``pad`` -- padding byte (optional) 

**Returns**: 

	0 if all OK 


Stand-alone script ``intelhex.py``
**********************************
You can use intelhex.py as stand-alone hex-to-bin convertor.
::

	Usage:
	    python intelhex.py [options] file.hex [out.bin]

	Arguments:
	    file.hex                name of hex file to processing.
	    out.bin                 name of output file.
				    If omitted then output write to file.bin.

	Options:
	    -h, --help              this help message.
	    -p, --pad=FF            pad byte for empty spaces (hex value).
	    -r, --range=START:END   specify address range for writing output
				    (hex value).
				    Range can be in form 'START:' or ':END'.
	    -l, --length=NNNN,
	    -s, --size=NNNN         size of output (decimal value).

Per example, converting content of foo.hex to foo.bin addresses from 0 to FF::

	$ python intelhex.py -r 0000:00FF foo.hex

Or (equivalent)::

	$ python intelhex.py -r 0000: -s 256 foo.hex
