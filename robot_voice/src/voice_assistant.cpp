/*
* 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的
* 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的
* 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <geometry_msgs/Twist.h>

#include "robot_voice/qtts.h"
#include "robot_voice/msp_cmn.h"
#include "robot_voice/msp_errors.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

int SPEED = 0.7;
/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF"
	int		size_8;                 // = FileSize - 8
	char            wave[4];                // = "WAVE"
	char            fmt[4];                 // = "fmt "
	int		fmt_size;		// = 下一个结构体的大小 : 16

	short int       format_tag;             // = PCM : 1
	short int       channels;               // = 通道数 : 1
	int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int		data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
	{ 'R', 'I', 'F', 'F' },
	0,
	{'W', 'A', 'V', 'E'},
	{'f', 'm', 't', ' '},
	16,
	1,
	1,
	16000,
	32000,
	2,
	16,
	{'d', 'a', 't', 'a'},
	0  
};
/* 文本合成 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)
	{
		printf("params is error!\n");
		return ret;
	}
	fp = fopen(des_path, "wb");
	if (NULL == fp)
	{
		printf("open %s error.\n", des_path);
		return ret;
	}
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionBegin failed, error code: %d.\n", ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSTextPut failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	printf("正在合成 ...\n");
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1) 
	{
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data)
		{
			fwrite(data, audio_len, 1, fp);
		    wav_hdr.data_size += audio_len; //计算data_size大小
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status)
			break;
		printf(">");
		usleep(150*1000); //防止频繁占用CPU
	}
	printf("\n");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSAudioGet failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "AudioGetError");
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
	
	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
	fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionEnd failed, error code: %d.\n",ret);
	}

	return ret;
}

std::string to_string(int val) 
{
    char buf[20];
    sprintf(buf, "%d", val);
    return std::string(buf);
}

int left_flag = 0;
int right_flag = 0;
int back_flag = 0;
int forward_flag = 0;
int stop_flag = 0;

void voiceWordsCallback(const std_msgs::String::ConstPtr& msg)
{
    char cmd[2000];
    const char* text;
    int         ret                  = MSP_SUCCESS;
    const char* session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
    const char* filename             = "tts_sample.wav"; //合成的语音文件名称

    std::cout<<"I heard :"<<msg->data.c_str()<<std::endl;
    text = msg->data.c_str();

    std::string dataString = msg->data;
    left_flag = 0;
    right_flag = 0;
    back_flag = 0;
    forward_flag = 0;
    stop_flag = 0;
    char replyString[100] = "Ok";
    char nonString[100] = "I don't understand your command";
    if(dataString.find("left") != std::string::npos 
    || dataString.find("left") != std::string::npos)
    {
        //char nameString[100] = "我是你的语音小助手，你可以叫我小R";
        //text = nameString;
        //std::cout<<text<<std::endl;
        left_flag = 1;
        
    }
    else if(dataString.find("right") != std::string::npos )
    {
        
        //text = eageString;
        //std::cout<<text<<std::endl;
        right_flag = 1;
    }
    else if(dataString.find("forward") != std::string::npos || dataString.find("keep") != std::string::npos)
    {
        //char helpString[100] = "你可以问我现在时间";
        //text = helpString;
        //std::cout<<text<<std::endl;
        forward_flag = 1;
    }
    else if(dataString.find("back") != std::string::npos || dataString.find("backwark") != std::string::npos)
    {
        //获取当前时间
        /*struct tm *ptm; 
        long ts; 

        ts = time(NULL); 
        ptm = localtime(&ts); 
        std::string string = "现在时间" + to_string(ptm-> tm_hour) + "点" + to_string(ptm-> tm_min) + "分";

        char timeString[40] = {0};
        string.copy(timeString, sizeof(string), 0);
        text = timeString;
        std::cout<<text<<std::endl;*/
        back_flag = 1;
    }
    else if(dataString.find("Stop") != std::string::npos || dataString.find("stop") != std::string::npos)
    {
        stop_flag = 1;     
    }
    //else
    //{
        //text = msg->data.c_str();
        
    //}
    /*if (left_flag||right_flag||back_flag||forward_flag||stop_flag){
    /* 文本合成 
        text = replyString;
    }
    else {
        text = nonString;
    }
    printf("开始合成 ...\n");
    ret = text_to_speech(text, filename, session_begin_params);
    if (MSP_SUCCESS != ret)
    {
        printf("text_to_speech failed, error code: %d.\n", ret);
    }
    printf("合成完毕\n");
 
    popen("play tts_sample.wav","r");*/
}

void toExit()
{
    printf("按任意键退出 ...\n");
    getchar();
    MSPLogout(); //退出登录
}

int main(int argc, char* argv[])
{
	int         ret                  = MSP_SUCCESS;
	const char* login_params         = "appid = 5f6ef0ad, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	/*
	* rdn:           合成音频数字发音方式
	* volume:        合成音频的音量
	* pitch:         合成音频的音调
	* speed:         合成音频对应的语速
	* voice_name:    合成发音人
	* sample_rate:   合成音频采样率
	* text_encoding: 合成文本编码格式
	*
	*/
	//const char* session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
	//const char* filename             = "tts_sample.wav"; //合成的语音文件名称
	//const char* text                 = "亲爱的用户，您好，这是一个语音合成示例，感谢您对科大讯飞语音技术的支持！科大讯飞是亚太地区最大的语音上市公司，股票代码：002230"; //合成文本

	/* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://www.xfyun.cn注册获取
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);
		//goto exit ;//登录失败，退出登录
        toExit();
	}
	printf("\n###########################################################################\n");
	printf("## 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的 ##\n");
	printf("## 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的  ##\n");
	printf("## 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。  ##\n");
	printf("###########################################################################\n\n");

    ros::init(argc,argv,"VoiceAssistant");
    ros::NodeHandle n;
    ros::Subscriber sub =n.subscribe("voiceWords", 1000, voiceWordsCallback);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Rate loop_rate(10);
    
    while (ros::ok()){
        geometry_msgs::Twist msg;
        ros::spinOnce();
        loop_rate.sleep();
        //int count = 0;
        if(left_flag){
            msg.linear.x = 1;
            msg.angular.z = -1;
            pub.publish(msg);
            //left_flag = 0;
        }
        else if(right_flag){
            msg.linear.x = 1;
            msg.angular.z = 1;
            pub.publish(msg);
            //right_flag = 0;
        }
        else if(forward_flag){
            msg.linear.x = 1;
            msg.angular.z = 0;
            pub.publish(msg);
            //forward_flag = 0;
        }
        else if(back_flag){
            msg.linear.x = -1;
            msg.angular.z = 0;
            pub.publish(msg);
            //back_flag = 0;
        }
        else if(stop_flag){
            msg.linear.x = 0;
            msg.angular.z = 0;
            pub.publish(msg);
            //stop_flag = 0;
        }

    }

    

exit:
	printf("按任意键退出 ...\n");
	getchar();
	MSPLogout(); //退出登录

	return 0;
}

