import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import pygame
from marv_msgs.msg import Keyboard
from std_msgs.msg import Int8

# Keys, {ord values : pygame key id}
keys = {\
        48 : 'k_0',\
        49 : 'k_1',\
        50 : 'k_2',\
        51 : 'k_3',\
        52 : 'k_4',\
        53 : 'k_5',\
        54 : 'k_6',\
        55 : 'k_7',\
        56 : 'k_8',\
        57 : 'k_9',\
        38 : 'k_ampersand',\
        42 : 'k_asterisk',\
        64 : 'k_at',\
        96 : 'k_backquote',\
        92 : 'k_backslash',\
        8 : 'k_backspace',\
        318 : 'k_break',\
        301 : 'k_capslock',\
        94 : 'k_caret',\
        12 : 'k_clear',\
        58 : 'k_colon',\
        44 : 'k_comma',\
        127 : 'k_delete',\
        36 : 'k_dollar',\
        1073741905 : 'k_down',\
        279 : 'k_end',\
        61 : 'k_equals',\
        27 : 'k_escape',\
        321 : 'k_euro',\
        33 : 'k_exclaim',\
        282 : 'k_f1',\
        291 : 'k_f10',\
        292 : 'k_f11',\
        293 : 'k_f12',\
        294 : 'k_f13',\
        295 : 'k_f14',\
        296 : 'k_f15',\
        283 : 'k_f2',\
        284 : 'k_f3',\
        285 : 'k_f4',\
        286 : 'k_f5',\
        287 : 'k_f6',\
        288 : 'k_f7',\
        289 : 'k_f8',\
        290 : 'k_f9',\
        0 : 'k_first',\
        62 : 'k_greater',\
        35 : 'k_hash',\
        315 : 'k_help',\
        278 : 'k_home',\
        277 : 'k_insert',\
        256 : 'k_kp0',\
        257 : 'k_kp1',\
        258 : 'k_kp2',\
        259 : 'k_kp3',\
        260 : 'k_kp4',\
        261 : 'k_kp5',\
        262 : 'k_kp6',\
        263 : 'k_kp7',\
        264 : 'k_kp8',\
        265 : 'k_kp9',\
        267 : 'k_kp_divide',\
        271 : 'k_kp_enter',\
        272 : 'k_kp_equals',\
        269 : 'k_kp_minus',\
        268 : 'k_kp_multiply',\
        266 : 'k_kp_period',\
        270 : 'k_kp_plus',\
        308 : 'k_lalt',\
        323 : 'k_last',\
        306 : 'k_lctrl',\
        1073741904 : 'k_left',\
        91 : 'k_leftbracket',\
        40 : 'k_leftparen',\
        60 : 'k_less',\
        310 : 'k_lmeta',\
        304 : 'k_lshift',\
        311 : 'k_lsuper',\
        319 : 'k_menu',\
        45 : 'k_minus',\
        313 : 'k_mode',\
        300 : 'k_numlock',\
        281 : 'k_pagedown',\
        280 : 'k_pageup',\
        19 : 'k_pause',\
        46 : 'k_period',\
        43 : 'k_plus',\
        320 : 'k_power',\
        316 : 'k_print',\
        63 : 'k_question',\
        39 : 'k_quote',\
        34 : 'k_quotedbl',\
        307 : 'k_ralt',\
        305 : 'k_rctrl',\
        13 : 'k_return',\
        1073741903 : 'k_right',\
        93 : 'k_rightbracket',\
        41 : 'k_rightparen',\
        309 : 'k_rmeta',\
        303 : 'k_rshift',\
        312 : 'k_rsuper',\
        302 : 'k_scrollock',\
        59 : 'k_semicolon',\
        47 : 'k_slash',\
        32 : 'k_space',\
        317 : 'k_sysreq',\
        9 : 'k_tab',\
        95 : 'k_underscore',\
        0 : 'k_unknown',\
        1073741906 : 'k_up',\
        97 : 'k_a',\
        98 : 'k_b',\
        99 : 'k_c',\
        100 : 'k_d',\
        101 : 'k_e',\
        102 : 'k_f',\
        103 : 'k_g',\
        104 : 'k_h',\
        105 : 'k_i',\
        106 : 'k_j',\
        107 : 'k_k',\
        108 : 'k_l',\
        109 : 'k_m',\
        110 : 'k_n',\
        111 : 'k_o',\
        112 : 'k_p',\
        113 : 'k_q',\
        114 : 'k_r',\
        115 : 'k_s',\
        116 : 'k_t',\
        117 : 'k_u',\
        118 : 'k_v',\
        119 : 'k_w',\
        120 : 'k_x',\
        121 : 'k_y',\
        122 : 'k_z'\
}

# Topics to publish
TOPICS = ['keyboard/binary', 'keyboard/keyup', 'keyboard/keydown']

class KeyboardStatePublisher(Node):
    """Publishes state of the keyboard."""

    def __init__(self):
        super().__init__('keyboard_input')

        self.msg_binary = Keyboard()
        self.pub = {}
        for topic in TOPICS:
            self.pub[topic] = self.create_publisher(Keyboard, topic, 10)
        
        #self.scenario_config_timer = self.create_timer(0.04, self.update)

    def update(self):
        # Init msgs
        msg_keyup = Keyboard()
        msg_keydown = Keyboard()

        # Transfer keyboard state in pygame -> msgs
        for ev in pygame.event.get():
            if ev.type == pygame.KEYUP:
                k = keys[ev.key]
                #if k is "k_up":
                #print(keys[ev.key])
                setattr(self.msg_binary, k, 0)

            if ev.type == pygame.KEYDOWN:
                k = keys[ev.key]
                #if k is "k_up":
                #print(keys[ev.key])
                setattr(self.msg_binary, k, 1)

        # Publish msgs
        for topic, msg in zip(TOPICS, [self.msg_binary, msg_keyup, msg_keydown]):
            self.pub[topic].publish(msg)


def main(args=None):
    pygame.init()
    pgscreen=pygame.display.set_mode((1, 1))
    pygame.display.set_caption('keyboard')

    rclpy.init(args=args)

    publisher = KeyboardStatePublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()