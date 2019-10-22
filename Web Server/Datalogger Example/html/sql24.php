<?php 
header('Content-Type: application/json');


//Open Database
class RPiHatDB extends SQLite3 {
	
	function __construct() {
		$this->open('RPiHat.db');
	}
	
}
$sqldb = new RPiHatDB();

if(!db){
	echo "Database error";
	exit();
}
//Query table data
$data = $sqldb->query('SELECT TEMP1,IR,FULLS,VIS,LUX,TEMP2,PRESSURE,HUMID,TIME FROM RPiHatData LIMIT 1440');
if(!data){
	echo "Table error";
	exit();
}

//Format JSON data
$array = array();
while ($currentrow = $data->fetchArray(SQLITE3_ASSOC)){
	$array['t'] = $currentrow['TIME'];
	$array['y'] = $currentrow['TEMP1'];
	$out['temp1'][] = $array;
}
while ($currentrow = $data->fetchArray(SQLITE3_ASSOC)){
	$array['t'] = $currentrow['TIME'];
	$array['y'] = $currentrow['IR'];
	$out['ir'][] = $array;
}
while ($currentrow = $data->fetchArray(SQLITE3_ASSOC)){
	$array['t'] = $currentrow['TIME'];
	$array['y'] = $currentrow['FULLS'];
	$out['full'][] = $array;
}
while ($currentrow = $data->fetchArray(SQLITE3_ASSOC)){
	$array['t'] = $currentrow['TIME'];
	$array['y'] = $currentrow['VIS'];
	$out['vis'][] = $array;
}
while ($currentrow = $data->fetchArray(SQLITE3_ASSOC)){
	$array['t'] = $currentrow['TIME'];
	$array['y'] = $currentrow['LUX'];
	$out['lux'][] = $array;
}
while ($currentrow = $data->fetchArray(SQLITE3_ASSOC)){
	$array['t'] = $currentrow['TIME'];
	$array['y'] = $currentrow['TEMP2'];
	$out['temp2'][] = $array;
}
while ($currentrow = $data->fetchArray(SQLITE3_ASSOC)){
	$array['t'] = $currentrow['TIME'];
	$array['y'] = $currentrow['PRESSURE'];
	$out['pressure'][] = $array;
}
while ($currentrow = $data->fetchArray(SQLITE3_ASSOC)){
	$array['t'] = $currentrow['TIME'];
	$array['y'] = $currentrow['HUMID'];
	$out['humidity'][] = $array;
}

//Output JSON data
echo json_encode($out);

?>
