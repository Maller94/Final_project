import sys, pygame
pygame.init()

WIDTH, HEIGHT = 600, 600
win = pygame.display.set_mode((WIDTH,HEIGHT))
pygame.display.set_caption('Robot Simulator')

def quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()


def main():
    clock = pygame.time.Clock()
    # The arena
    BLUE = (0,0,255)
    while True:
        quit()
        clock.tick(60)
        # Init white color
        win.fill((255,255,255))
        pygame.draw.rect(win,(0,0,255),[10,10,580,580],2)
        pygame.display.update()

if __name__ == '__main__':
    main()
