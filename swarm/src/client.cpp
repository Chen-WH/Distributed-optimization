#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <time.h>

int main(){
		clock_t start, end;
		start = clock();
    //向服务器（特定的IP和端口）发起请求
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));  //每个字节都用0填充
    serv_addr.sin_family = AF_INET;  //使用IPv4地址
    serv_addr.sin_addr.s_addr = inet_addr("192.168.3.154");  //具体的服务器端IP地址
    serv_addr.sin_port = htons(1234);  //端口
		char buffer[] = "Hello World!";
		char bufRecv[40] = {0};
    for (int i = 0; i < 100; ++i){
						int sock = socket(AF_INET, SOCK_STREAM, 0);
						connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
						write(sock, buffer, sizeof(buffer));
						read(sock, bufRecv, sizeof(bufRecv)-1);
						printf("Message form server: %s\n", bufRecv);
						memset(bufRecv, 0, 40);
						close(sock);
		}
		end = clock();
		printf("运行时间%lfms\n", (double)(end - start) / CLOCKS_PER_SEC * 1000);
    return 0;
}
