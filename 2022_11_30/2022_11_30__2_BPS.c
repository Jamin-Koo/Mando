#include <stdio.h>

int main() {

	float BPS, Byte, byte_ms, byte_input;
	int inputBPS, inputByte;

	int BPS_num[] = { 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };

	printf("BPS �Է� : (1): 1200, (2): 2400, (3): 4800, (4): 9600, (5): 14400, \n(6): 19200, (7): 38400, (8)57600, (9): 115200, (10): 230400, (11): 460800, (12): 921600\n");
	scanf_s("%d", &inputBPS);

	inputBPS = BPS_num[inputBPS - 1];
	printf("%d \n", inputBPS);

	printf("��� byte �Է� : ");
	scanf_s("%d", &inputByte);
	printf("%d\n", inputByte);

	BPS = inputBPS / 1000;
	Byte = inputByte;

	byte_ms = 1 / BPS;
	byte_input = Byte * byte_ms;


	printf("1 byte�� �ҿ�ð� =  % 0.3f ms,  �Է� byte�� �´� �ð� = % 0.3f ms�Դϴ�.", byte_ms, byte_input);

	return 0;
}