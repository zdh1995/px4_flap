/* 
 * 串口读取超声波
 * 
 */
#include <px4_platform_common/tasks.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int ultrasonic_distance_task; /**< Handle of deamon task / thread */

__EXPORT int ultrasonic_distance_main(int argc, char *argv[]);

static int uart_init(char * uart_name);
static int ultrasonic_distance_thread_main(int argc, char *argv[]);
static int set_uart_baudrate(const int fd, unsigned int baud);
static void usage(const char * reason);

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* 以新的配置填充结构体 */
    /* 设置某个选项，那么就使用"|="运算，
     * 如果关闭某个选项就使用"&="和"~"运算
     * */
    tcgetattr(fd, &uart_config); // 获取终端参数

    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

    /* 无偶校验，一个停止位 */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB 使用两个停止位，PARENB 表示偶校验

     /* 设置波特率 */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }
    // 设置与终端相关的参数，TCSANOW 立即改变参数
    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    /*Linux中，万物皆文件，打开串口设备和打开普通文件一样，使用的是open（）系统调用*/
    // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
//    printf("Open the %s\n",serial_fd);
    return serial_fd;
}

static void usage(const char * reason)
{
	if (reason){
		fprintf(stderr,"%s\n",reason);	
	}
	fprintf(stderr,"usage: position_estimator_inav {start|stop|status} [-v]\n");
}

int ultrasonic_distance_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}
	if (!strcmp(argv[1],"start")){
		if (thread_running){
			warnx("already running\n");
			return 0;		
		}
		thread_should_exit = false ;
		ultrasonic_distance_task = px4_task_spawn_cmd("ultrasonic_distance",
                                          SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 2000,
                                          ultrasonic_distance_thread_main,
                                          (argv) ? (char *const *) &argv[2] : (char *const *) NULL);
		return 0;
	}
	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int ultrasonic_distance_thread_main(int argc, char *argv[])
{
    char data;
    int step= 1 ;
    int output1 = 0xe8;
    int output2 = 0x02;
    int output3 = 0xbc;
	int distance;
    int ret;
	float distance_m;
    //double distance_m;
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_read = uart_init("/dev/ttyS6");
    if(false == uart_read)
        return -1;
    if(false == set_uart_baudrate(uart_read,9600)){
        printf("set_uart_baudrate is failed\n");
        return -1;
    }
    printf("uart init is successful\n");
	thread_running = true;

	struct distance_sensor_s report;
	memset(&report,0,sizeof(report));

	orb_advert_t distance_sensor_pub = orb_advertise(ORB_ID(distance_sensor),&report);

    while(!thread_should_exit){
        if (step == 1){
			ret = write(uart_read,&output1,1);
			usleep(40);
			step++;
		}else if (step == 2){
			ret = write(uart_read,&output2,1);
			usleep(40);
			step++;
		}else if (step == 3){
			ret = write(uart_read,&output3,1);
			usleep(40);
			step++;
		}else if (step == 4){
			    distance = 0;
			    for(int i = 0;i <2;++i){
				data='0';
                ret = read(uart_read,&data,1);
                if (ret < 0)
                {
                    PX4_INFO("ERR: %d\n",ret);
                } else if (ret == 0)
                {
                    PX4_INFO("EOF");
                }
				distance=distance*i*256+data;
			    }
			    distance_m = (float) distance / 1000;
			    report.current_distance = distance_m;
			    report.max_distance=7.65f;
			    report.min_distance=0.05f;
			    report.timestamp=hrt_absolute_time();

			    orb_publish(ORB_ID(distance_sensor),distance_sensor_pub,&report);

			    //distance_m = (double)distance / 1000;
			    
			    //printf("distance: %f\n",distance_m);
			    step = 1;
			    
        	}
    	}
	warnx("exiting");
	thread_running = false;
	close(uart_read);
    return 0;
}
