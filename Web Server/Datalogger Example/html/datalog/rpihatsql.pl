#!/usr/bin/perl

use strict;
use Device::SerialPort;
use DBI;

# SERIAL PORT SETUP
my $port = "/dev/ttyACM0";
my $obj = new Device::SerialPort($port) || die "Can't open $port";

$obj->baudrate(115200);
$obj->databits(8);
$obj->parity("none");
$obj->stopbits(1);

# SETUP SQL INTERFACE
my $driver = "SQLite";
my $db = "/var/www/html/RPiHat.db";
my $dsn = "DBI:$driver:dbname=$db";
my $userid = "";
my $password = "";
my $dbh = DBI->connect($dsn, $userid, $password, { RaiseError => 1})
		|| die $DBI::errstr;

print "Opened RPiHat DB\n";

$obj->write("K");
my $reply = $obj->read(111);
print "$reply\n";

# Remove bars and spaces
$reply =~ s/\| //g;
print "$reply\n";

# Retrieve data from reply and store in appropriate variables
my ($temp1, $ir, $fulls, $vis, $lux, $temp2, $pressure, $humid) = split / /, $reply;

# Create entry for RPiHat DB
my $entry = qq(INSERT INTO RPiHatData(TEMP1,IR,FULLS,VIS,LUX,TEMP2,PRESSURE,HUMID,TIME) 
	VALUES($temp1,$ir,$fulls,$vis,$lux,$temp2,$pressure,$humid,datetime('now','localtime')));

# Enter entry into RPiHat DB
my $rv = $dbh->do($entry) or die $DBI::errstr;
print "Data entry successfully\n";

# Disconnect from RPiHat DB
$dbh->disconnect();
