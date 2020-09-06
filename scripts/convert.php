<?php

if($argc > 1){
$file = $argv[1];
$row_count = (array_key_exists(2, $argv))?$argv[2]:-1;

$fp = fopen($file, 'rb');
$contents = fread($fp, filesize($file));
fclose($fp);

$converted = bin2hex($contents);
$byte_count = ceil(strlen($converted) / 2);

$numlines = 0;
echo 'db ';
for($i=0;$i < $byte_count;$i += 2){
	echo '0x'.substr($converted, $i, 2);
	echo (($i + 2) % 32 == 0)?' ':', ';
 	if(($i + 2) % 32 == 0) {
		if($row_count != -1 && $row_count == ++$numlines){
			echo "\n";
			break;
		}
		//echo "\033[2J";
		echo "\n";
		echo 'db ';
	}
}
}else{
	echo "Usage: <script name> <bin file>\n";
}

