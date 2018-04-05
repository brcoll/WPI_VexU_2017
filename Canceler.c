// Flags for control loops to automatically exit
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
}
