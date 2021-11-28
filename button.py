import pygame

from globals import HOVER_GRAY_COLOR, BLACK


class Button:
    """Create a button and allows to draw it on screen"""

    def __init__(self, text, pos, size, color, surface, font_size=16):
        self.text = text
        self.pos = pos
        self.size = size
        self.color = color
        self.surface = surface
        print(size)
        self.dim_x = (self.pos[0], self.pos[0] + self.size[0])
        self.dim_y = (self.pos[1], self.pos[1] + self.size[1])

        self.font = pygame.font.SysFont("Arial", font_size, bold=True)

    def show(self):
        mouse = pygame.mouse.get_pos()

        if self.pos[0] <= mouse[0] <= self.pos[0] + self.size[0] and \
                self.pos[1] <= mouse[1] <= self.pos[1] + self.size[1]:
            # HOVER effect on button
            pygame.draw.rect(self.surface, HOVER_GRAY_COLOR, [self.pos[0], self.pos[1],
                                                        self.size[0], self.size[1]],
                             border_radius=5)
        else:
            pygame.draw.rect(self.surface, self.color, [self.pos[0], self.pos[1],
                                                        self.size[0], self.size[1]],
                             border_radius=5)

        # required for multiple lines text in button
        y = self.pos[1] + self.size[1] / 2 if len(self.text) == 1 else self.pos[1] + self.size[1] / 2.5
        for line in self.text:
            text = self.font.render(line, True, BLACK)  # define text in black
            text_rect = text.get_rect(center=(self.pos[0] + self.size[0] / 2, y))
            self.surface.blit(text, text_rect)
            y += text_rect.height

    def is_hover(self):
        mouse = pygame.mouse.get_pos()
        return self.dim_x[0] <= mouse[0] <= self.dim_x[1] and self.dim_y[0] <= mouse[1] <= self.dim_y[1]