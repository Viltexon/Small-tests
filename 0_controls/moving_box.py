import pygame
import rospy

from geometry_msgs.msg import Point


class Player(pygame.sprite.Sprite):
    def __init__(self, player_dim, velocity, player_color):
        pygame.sprite.Sprite.__init__(self)

        self.player_dim = player_dim
        self.image = pygame.Surface((self.player_dim, self.player_dim))
        self.image.fill(player_color)
        self.rect = self.image.get_rect()
        self.rect.center = (self.player_dim, self.player_dim)
        self.velocity = velocity
        self.speedX = 0
        self.speedY = 0

    def update(self, screen_dim, direction):

        self.speedX = 0
        self.speedY = 0
        if self.rect.x + self.velocity * direction[0] > 0:
            if self.rect.x + self.velocity * direction[0] < screen_dim - self.player_dim:
                self.speedX = self.velocity * direction[0]

        if self.rect.y + self.velocity * direction[1] > 0:
            if self.rect.y + self.velocity * direction[1] < screen_dim - self.player_dim:
                self.speedY = self.velocity * direction[1]

        self.rect.x += self.speedX
        self.rect.y += self.speedY


class WorldNode(object):
    def __init__(self):
        self.screen_color = (0, 0, 0)
        self.screen_dim = 500

        self.direction = [0, 0]

        rospy.init_node('WorldNode')
        rospy.Subscriber('direction', Point, self.direction_callback)

    def direction_callback(self, direction):
        self.direction[0] = direction.x
        self.direction[1] = direction.y

    def run(self):
        pygame.init()

        screen = pygame.display.set_mode((self.screen_dim, self.screen_dim))
        pygame.display.set_caption("WorldNode")

        velocity = 5
        player_dim = 50
        player_color = (0, 255, 0)

        all_sprites = pygame.sprite.Group()
        player = Player(player_dim, velocity, player_color)
        all_sprites.add(player)

        refresh_rate = 25

        rate = rospy.Rate(refresh_rate)
        done = False
        while not rospy.is_shutdown() and not done:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            all_sprites.update(self.screen_dim, self.direction)

            screen.fill(self.screen_color)
            all_sprites.draw(screen)
            pygame.display.flip()

            rate.sleep()


if __name__ == "__main__":
    node = WorldNode()
    node.run()

