#include <stdio.h>

int main() {

	float BPS, Byte, byte_ms, byte_input;
	int inputBPS, inputByte;

	printf("BPS �Է� : ");
	scanf_s("%d", &inputBPS);
	printf("%d\n", inputBPS);

	printf("��� byte �Է� : ");
	scanf_s("%d", &inputByte);
	printf("%d\n", inputByte);

	BPS = inputBPS/1000;
	Byte = inputByte;

	byte_ms = 1 / BPS;
	byte_input = Byte * byte_ms;


	printf("1 byte�� �ҿ�ð� =  % 0.3f ms,  �Է� byte�� �´� �ð� = % 0.3f ms�Դϴ�.", byte_ms, byte_input);

	return 0;
}