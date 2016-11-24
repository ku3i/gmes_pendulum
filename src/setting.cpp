
#include "./setting.h"

Setting::Setting( int argc, char **argv )
{
        read_options(argc, argv);
}

void
Setting::read_options(int argc, char **argv)
{
    for (int i = 1; i < argc; ++i)
  	{
		if ((strncmp(argv[i], "--help", 6) == 0) || (strncmp(argv[i], "-h", 2) == 0))
		{
            printf("Help, Options: \n");
            //printf("    -p  --port     change TCP port\n");
            //printf("    -h  --help     display this help\n");
            printf("\n");
            exit(0);
		}
//		else if ((strncmp(argv[i], "--port", 6) == 0) || (strncmp(argv[i], "-p", 2) == 0))
//		{
//            if (argc < i+2)
//            {
//                printf("usage: %s -p <port>\n", argv[0]);
//                exit(0);
//            }
//            else
//            {
//                tcp_port = atoi(argv[i+1]);
//                ++i;
//            }
//		}
//		else if ((strncmp(argv[i], "--robot", 7) == 0) || (strncmp(argv[i], "-r", 2) == 0))
//		{
//            if (argc < i+2)
//            {
//                printf("usage: %s -r <ID>\n", argv[0]);
//                exit(0);
//            }
//            else
//            {
//                robot_ID = atoi(argv[i+1]);
//                ++i;
//            }
//		}
    }
}

