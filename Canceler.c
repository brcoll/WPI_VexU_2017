
task canceler()
{
	while (true){
		if (vexRT[Btn7D]){
			disabled = true;
		}
		else {
			disabled = false;
		}
	}
	delay(50);
}
