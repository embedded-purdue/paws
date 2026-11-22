/* Interactive UDP client for macOS/Linux (POSIX sockets).
 *
 * Type a line, press Enter, it's sent to the ESP32-P4 UDP echo server;
 * whatever comes back is printed. Repeats until EOF (Ctrl+D) or you type
 * "quit"/"exit".
 *
 * Usage:
 *   ./udp_interactive_client 192.168.7.2            # default port 3333
 *   ./udp_interactive_client 192.168.7.2 3333
 */
#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_PORT     3333
#define MAX_LINE         512 /* matches the P4 firmware's MAX_PACKET */
#define RECV_TIMEOUT_SEC 2

/* A previous program (vim, ncurses, a crashed TUI, ...) can leave the
 * terminal in raw/no-echo mode, which makes fgets() look "broken" even
 * though this program never touched termios. Re-assert canonical line
 * editing, local echo, and signal generation (Ctrl+C/Ctrl+D) before we
 * read anything, so typing always behaves normally. No-op when stdin
 * isn't a terminal (piped input, redirected file). */
static void ensure_sane_terminal(void)
{
    if (!isatty(STDIN_FILENO)) {
        return;
    }
    struct termios t;
    if (tcgetattr(STDIN_FILENO, &t) != 0) {
        return;
    }
    t.c_lflag |= (ICANON | ECHO | ECHOE | ISIG);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

static double elapsed_ms(const struct timespec *start, const struct timespec *end)
{
    return (double)(end->tv_sec - start->tv_sec) * 1000.0 +
           (double)(end->tv_nsec - start->tv_nsec) / 1000000.0;
}

/* Read one line into `line` (capacity `cap`), stripping the trailing
 * newline. Returns line length, 0 for a blank line, or -1 on EOF. If the
 * typed line was longer than the buffer, discards the overflow from stdin
 * so it doesn't leak into the next read, and warns on stderr. */
static ssize_t read_line(char *line, size_t cap)
{
    if (!fgets(line, (int)cap, stdin)) {
        return -1;
    }

    size_t len = strlen(line);
    if (len == cap - 1 && line[len - 1] != '\n') {
        int c;
        while ((c = getchar()) != '\n' && c != EOF) {
            /* discard the rest of an over-long line */
        }
        fprintf(stderr, "Warning: input line exceeded %zu bytes; truncated.\n", cap - 1);
    }

    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        line[--len] = '\0';
    }
    return (ssize_t)len;
}

int main(int argc, char **argv)
{
    if (argc != 2 && argc != 3) {
        fprintf(stderr, "Usage: %s <p4-ip> [port]\n", argv[0]);
        return 1;
    }

    const char *ip = argv[1];
    long port = (argc == 3) ? strtol(argv[2], NULL, 10) : DEFAULT_PORT;
    if (port <= 0 || port > 65535) {
        fprintf(stderr, "Invalid port: %s\n", argv[2]);
        return 1;
    }

    ensure_sane_terminal();
    setvbuf(stdout, NULL, _IOLBF, 0); /* flush every line, even if stdout is redirected */

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in p4 = {0};
    p4.sin_family = AF_INET;
    p4.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip, &p4.sin_addr) != 1) {
        fprintf(stderr, "Invalid IPv4 address: %s\n", ip);
        close(sock);
        return 1;
    }

    printf("Target %s:%ld. Type a message and press Enter; 'quit'/'exit' or Ctrl+D stops.\n\n",
           ip, port);

    char line[MAX_LINE];
    for (;;) {
        printf("> ");
        fflush(stdout);

        ssize_t len = read_line(line, sizeof(line));
        if (len < 0) {
            printf("\nEOF on stdin, exiting.\n");
            break;
        }
        if (len == 0) {
            continue; /* blank line, nothing to send */
        }
        if (strcmp(line, "quit") == 0 || strcmp(line, "exit") == 0) {
            break;
        }

        struct timespec before, after;
        clock_gettime(CLOCK_MONOTONIC, &before);

        ssize_t sent = sendto(sock, line, (size_t)len, 0, (struct sockaddr *)&p4, sizeof(p4));
        if (sent < 0) {
            perror("sendto");
            continue;
        }
        if (sent != len) {
            fprintf(stderr, "Short send: sent %zd of %zd bytes\n", sent, len);
            continue;
        }

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sock, &readfds);
        struct timeval timeout = { .tv_sec = RECV_TIMEOUT_SEC, .tv_usec = 0 };

        int ready = select(sock + 1, &readfds, NULL, NULL, &timeout);
        if (ready < 0) {
            perror("select");
            continue;
        }
        if (ready == 0) {
            printf("(no reply within %ds — check link/IP/firewall)\n", RECV_TIMEOUT_SEC);
            continue;
        }

        char received[MAX_LINE + 1];
        struct sockaddr_in source = {0};
        socklen_t source_len = sizeof(source);
        ssize_t n = recvfrom(sock, received, sizeof(received) - 1, 0,
                              (struct sockaddr *)&source, &source_len);
        clock_gettime(CLOCK_MONOTONIC, &after);
        if (n < 0) {
            perror("recvfrom");
            continue;
        }
        received[n] = '\0';

        char source_ip[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &source.sin_addr, source_ip, sizeof(source_ip));
        printf("< %.*s  [%zd bytes from %s:%u, %.2f ms]\n",
               (int)n, received, n, source_ip, (unsigned)ntohs(source.sin_port),
               elapsed_ms(&before, &after));
    }

    close(sock);
    return 0;
}
