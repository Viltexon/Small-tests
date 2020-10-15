import pygame
import rospy

from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray


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

    def update(self, screen_dim, player_color, view_color):

        self.speedX = 0
        self.speedY = 0
        keystate = pygame.key.get_pressed()
        if keystate[pygame.K_LEFT] and self.rect.x > 0:
            self.speedX = -self.velocity
        if keystate[pygame.K_RIGHT] and self.rect.x < screen_dim - self.player_dim:
            self.speedX = self.velocity
        if keystate[pygame.K_UP] and self.rect.y > 0:
            self.speedY = -self.velocity
        if keystate[pygame.K_DOWN] and self.rect.y < screen_dim - self.player_dim:
            self.speedY = self.velocity

        self.rect.x += self.speedX
        self.rect.y += self.speedY

        self.image.fill(player_color)

        zone_pub = rospy.Publisher('zone_color', UInt8MultiArray, queue_size=1)

        zone_color = UInt8MultiArray()
        zone_color.data = view_color
        zone_pub.publish(zone_color)

    def get_center(self):
        return self.rect.center


class WorldNode(object):
    def __init__(self):
        self.screen_color = (0, 0, 0)
        self.screen_dim = 500

        self.reflex = False

        rospy.init_node('WorldNode')
        rospy.Subscriber('body_color', Bool, self.body_color_callback)

    def body_color_callback(self, reflex):
        self.reflex = reflex.data

    def run(self):
        pygame.init()

        screen = pygame.display.set_mode((self.screen_dim, self.screen_dim))
        pygame.display.set_caption("WorldNode")

        velocity = 5
        player_dim = 50
        player_color_basic = (0, 255, 0)

        view_color = list(self.screen_color)

        all_sprites = pygame.sprite.Group()
        player = Player(player_dim, velocity, player_color_basic)
        all_sprites.add(player)

        refresh_rate = 25

        rate = rospy.Rate(refresh_rate)
        done = False
        while not rospy.is_shutdown() and not done:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            player_color = player_color_basic
            if self.reflex:
                player_color = (130, 0, 0)

            position = player.get_center()
            try:
                view_color_up = screen.get_at((position[0], position[1] - player_dim // 2 - 1))
                view_color_down = screen.get_at((position[0], position[1] + player_dim // 2 + 1))
                view_color_left = screen.get_at((position[0] - player_dim // 2 - 1, position[1]))
                view_color_right = screen.get_at((position[0] + player_dim // 2 + 1, position[1]))
                if view_color_up == view_color_down == view_color_left == view_color_right:
                    view_color = list(view_color_up)[0:3]
            except:
                pass

            all_sprites.update(self.screen_dim, player_color, view_color)

            screen.fill(self.screen_color)
            pygame.draw.rect(screen, (255, 0, 0), (300, 300, self.screen_dim, self.screen_dim))
            all_sprites.draw(screen)
            pygame.display.flip()

            rate.sleep()


if __name__ == "__main__":
    node = WorldNode()
    node.run()
