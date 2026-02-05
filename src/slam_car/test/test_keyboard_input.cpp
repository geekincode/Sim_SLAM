#include <iostream>
#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_I 0x69
#define KEYCODE_K 0x6b
#define KEYCODE_J 0x6a
#define KEYCODE_L 0x6c
#define KEYCODE_COMMA 0x2c
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_N 0x6E
#define KEYCODE_M 0x6D

class KeyboardTester
{
public:
    KeyboardTester()
    {
        std::cout << "键盘输入测试程序启动" << std::endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << "按任意键测试，Q键退出" << std::endl;
        std::cout << "---------------------------" << std::endl;
        std::cout << "预定义的键码映射:" << std::endl;
        std::cout << "W: 0x" << std::hex << KEYCODE_W << std::dec << " ('w')" << std::endl;
        std::cout << "A: 0x" << std::hex << KEYCODE_A << std::dec << " ('a')" << std::endl;
        std::cout << "S: 0x" << std::hex << KEYCODE_S << std::dec << " ('s')" << std::endl;
        std::cout << "D: 0x" << std::hex << KEYCODE_D << std::dec << " ('d')" << std::endl;
        std::cout << "Q: 0x" << std::hex << KEYCODE_Q << std::dec << " ('q')" << std::endl;
        std::cout << "E: 0x" << std::hex << KEYCODE_E << std::dec << " ('e')" << std::endl;
        std::cout << "I: 0x" << std::hex << KEYCODE_I << std::dec << " ('i')" << std::endl;
        std::cout << "K: 0x" << std::hex << KEYCODE_K << std::dec << " ('k')" << std::endl;
        std::cout << "J: 0x" << std::hex << KEYCODE_J << std::dec << " ('j')" << std::endl;
        std::cout << "L: 0x" << std::hex << KEYCODE_L << std::dec << " ('l')" << std::endl;
        std::cout << "COMMA: 0x" << std::hex << KEYCODE_COMMA << std::dec << " (',')" << std::endl;
        std::cout << "U: 0x" << std::hex << KEYCODE_U << std::dec << " ('u')" << std::endl;
        std::cout << "O: 0x" << std::hex << KEYCODE_O << std::dec << " ('o')" << std::endl;
        std::cout << "N: 0x" << std::hex << KEYCODE_N << std::dec << " ('n')" << std::endl;
        std::cout << "M: 0x" << std::hex << KEYCODE_M << std::dec << " ('m')" << std::endl;
        std::cout << "---------------------------" << std::endl;
    }

    int kfd = 0;
    struct termios cooked, raw;

    char getKey()
    {
        fd_set fds;
        int ret;
        char c = 0;

        FD_ZERO(&fds);
        FD_SET(kfd, &fds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000; // 10毫秒

        ret = select(kfd+1, &fds, NULL, NULL, &tv);

        if ((ret < 0) && (errno != EINTR))
        {
            perror("select error");
            exit(-1);
        }
        else if (ret == 0)
        {
            // 超时，没有按键
        }
        else
        {
            if (read(kfd, &c, 1) < 0)
            {
                perror("read error");
                exit(-1);
            }
        }

        return c;
    }

    void cleanup()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

    void runTest()
    {
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

        char c;
        bool running = true;

        while (running) {
            c = getKey();

            if (c != 0) {
                printf("按下了键: '%c' (十六进制: 0x%x, 十进制: %d)\n", c, (unsigned char)c, (unsigned char)c);

            // 将字符转换为小写以统一处理大写和小写字母
            char lower_c = std::tolower(c);

                // 检查是否是我们定义的键
                switch((unsigned char)lower_c) {
                    case KEYCODE_W:
                        std::cout << "  -> 这是 W 键" << std::endl;
                        break;
                    case KEYCODE_A:
                        std::cout << "  -> 这是 A 键" << std::endl;
                        break;
                    case KEYCODE_S:
                        std::cout << "  -> 这是 S 键" << std::endl;
                        break;
                    case KEYCODE_D:
                        std::cout << "  -> 这是 D 键" << std::endl;
                        break;
                    case KEYCODE_Q:
                        std::cout << "  -> 这是 Q 键，程序即将退出" << std::endl;
                        running = false;
                        break;
                    case KEYCODE_E:
                        std::cout << "  -> 这是 E 键" << std::endl;
                        break;
                    case KEYCODE_I:
                        std::cout << "  -> 这是 I 键 (前进)" << std::endl;
                        break;
                    case KEYCODE_K:
                        std::cout << "  -> 这是 K 键" << std::endl;
                        break;
                    case KEYCODE_J:
                        std::cout << "  -> 这是 J 键 (左转)" << std::endl;
                        break;
                    case KEYCODE_L:
                        std::cout << "  -> 这是 L 键 (右转)" << std::endl;
                        break;
                    case KEYCODE_COMMA:
                        std::cout << "  -> 这是 COMMA 键 (后退)" << std::endl;
                        break;
                    case KEYCODE_U:
                        std::cout << "  -> 这是 U 键 (左前)" << std::endl;
                        break;
                    case KEYCODE_O:
                        std::cout << "  -> 这是 O 键 (右前)" << std::endl;
                        break;
                    case KEYCODE_N:
                        std::cout << "  -> 这是 N 键 (左后)" << std::endl;
                        break;
                    case KEYCODE_M:
                        std::cout << "  -> 这是 M 键 (右后)" << std::endl;
                        break;
                    default:
                        std::cout << "  -> 这不是预定义的控制键" << std::endl;
                        break;
                }
            }

            usleep(10000); // 10毫秒延迟
        }

        tcsetattr(kfd, TCSANOW, &cooked);
    }
};

void quit(int sig)
{
    (void)sig;
    std::cout << "程序退出..." << std::endl;
    exit(0);
}

int main(int argc, char** argv)
{
    signal(SIGINT, quit);
    
    KeyboardTester tester;
    tester.runTest();
    
    return 0;
}

