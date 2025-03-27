#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#define TARGET "amcl"
#define TARGET_OUTPUT_D "/root/RESULT/%s/outputs/default/fuzzer_stats"
#define TARGET_RESULT_D "/root/RESULT/%s/cvg_hour/%s"
// year, month, day, hour, minute, second 
#define CVG_FILE_FORMAT "%04d/%02d/%02d\t%02d:%02d:%02d\nCoverage: %lld\nCrash: %lld\n\n"

#define CRASH_WORD "saved_crashes"
#define COVERAGE_WORD "edges_found"
char *TARGET_OUTPUT;
char *TARGET_RESULT;
char *TARGET_RESULT_CONTENT;

#define BUFFER_MAX_COUNT 300

void init(void);
void dnit(void);
void file_rw(void);
long long int find_change_s_to_lld(char buffer[BUFFER_MAX_COUNT]);

void init(void) {
	TARGET_OUTPUT = (char*)malloc(sizeof(char) * 50);
	TARGET_RESULT = (char*)malloc(sizeof(char) * 50);
	TARGET_RESULT_CONTENT = (char*)malloc(sizeof(char) * 200);
	if( !TARGET_OUTPUT || !TARGET_RESULT || !TARGET_RESULT_CONTENT )
	{
		exit(0);
	}
	memset(TARGET_OUTPUT, 0, 50);
	memset(TARGET_RESULT, 0, 50);
	memset(TARGET_RESULT_CONTENT, 0, 200);

	sprintf(TARGET_OUTPUT, TARGET_OUTPUT_D, TARGET);
	sprintf(TARGET_RESULT, TARGET_RESULT_D, TARGET, TARGET);
}
void dnit(void) {
	free(TARGET_OUTPUT);
	free(TARGET_RESULT);
	free(TARGET_RESULT_CONTENT);
}
/* 
tm_sec, tm_min, tm_hour, tm_mday, tm_mon + 1, tm_year + 1900
*/
void file_rw(void) {
	time_t t=time(NULL);
	struct tm cur_time = *localtime(&t);
	
	FILE* f_output = fopen(TARGET_OUTPUT, "rt");
	FILE* f_result = fopen(TARGET_RESULT, "at+");
	if( !f_output || !f_result) {
		exit(0);
	}
	fseek(f_output, 0, SEEK_END);
	long int file_size = ftell(f_output);
	fseek(f_output, 0, SEEK_SET);

	char buffer[BUFFER_MAX_COUNT] = {0,};
	long long int crashes = 0, coverages = 0;
	while(file_size > ftell(f_output)) {
		memset(buffer, 0, BUFFER_MAX_COUNT);
		fgets(buffer, BUFFER_MAX_COUNT, f_output);
		if( strncmp(buffer, CRASH_WORD, strlen(CRASH_WORD)) == 0 ) {
			crashes = find_change_s_to_lld(buffer);
		}
		else if( strncmp(buffer, COVERAGE_WORD, strlen(COVERAGE_WORD)) ==0 ) {
			coverages = find_change_s_to_lld(buffer);
		}

		if(crashes != 0 && coverages != 0) {
			break;
		}
	}
	
	// Debug
	// printf("%lld\n%lld\n", crashes, coverages);
	
	sprintf(TARGET_RESULT_CONTENT, CVG_FILE_FORMAT, 
			cur_time.tm_year + 1900,
			cur_time.tm_mon + 1,
			cur_time.tm_mday,
			cur_time.tm_hour,
			cur_time.tm_min,
			cur_time.tm_sec,
			coverages,
			crashes);

	fputs(TARGET_RESULT_CONTENT, f_result);
	
	fclose(f_output);
	fclose(f_result);
}

long long int find_change_s_to_lld(char buffer[BUFFER_MAX_COUNT]) {
	long long int ret = 0;

	for(char *p1=buffer; p1< buffer+BUFFER_MAX_COUNT; p1++) {
		if( *p1 == ':' ) {
			p1 += 2;
			for(char *p2=p1; p2 < buffer + BUFFER_MAX_COUNT; p2++) {
				if( *p2 == '\n' ) {
					break;
				}

				ret = (10 * ret) + (long long int)(*p2 - '0');
			}
			break;
		}
	}
	return ret;
}

int main(void) {
	init();
	
	file_rw();

	dnit();
}
