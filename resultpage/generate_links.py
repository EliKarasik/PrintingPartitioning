models135 = [('Airplane', '135/airplane_260621241.png', '135/airplane_260621242.png'),
             ('Bearing', '135/bearing_18579524.png', '135/bearing_18579525.png'),
             ('Cat', '135/cat_1127430868.png', '135/cat_1127430869.png'),
             ('Deer', '135/deer_1456820985.png', '135/deer_1456820987.png'),
             ('Dolphin', '135/dolphin_1014370948.png', '135/dolphin_1014370949.png'),
             ('Fertility', '135/FE_989486094.png', '135/FE_989486096.png'),
             ('Genus', '135/genus_1990237208.png', '135/genus_1990237210.png'),
             ('Helix', '135/helix_581263918.png', '135/helix_581263919.png'),
             ('Inuksuk', '135/Inuksuk_1622278277.png', '135/Inuksuk_1622278278.png'),
             ('Octopus', '135/octopus_1702848595.png', '135/octopus_1702848596.png'),
             ('Sculpture', '135/sculpture_1911635583.png', '135/sculpture_1911635584.png'),
             ('Tree', '135/tree_2021860158.png', '135/tree_2021860158.png')
             ]

models150 = [('Armadillo', '150/armadillo_1291920752.png', '150/armadillo_1291920753.png'),
             ('Fertility', '150/FE_1318948755.png', '150/FE_1318948756.png'),
             ('Isidore Horse', '150/isidore_horse_1159947873.png', '150/isidore_horse_1159947874.png'),
             ('Sculpture', '150/sculpture_1059925183.png', '150/sculpture_1059925184.png'),
             ('Gargoyle', '150/gargoyle_1263639035.png', '150/gargoyle_1263639036.png'),
             ]

template = """
<p class="bold" id='{0}{1}'>{1}</p>
<a href="{2}"><img class="thumbnail" src="{2}"></a>
<a href="{3}"><img class="thumbnail" src="{3}"></a>
<br/>
"""

link_template = "  <a href='#{0}{1}'>{1}</a>  |"

for prefix, models in [("cpa135", models135), ("cpa150", models150)]:
    for m in models:
        print(template.format(prefix, *m))

    links = ""
    for m in models:
        print(link_template.format(prefix, m[0]))
