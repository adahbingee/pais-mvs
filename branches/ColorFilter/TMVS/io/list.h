#ifndef __LIST_H
#define __LIST_H

typedef struct
{
	int item_count;
	int current_max_size;
	char growable;

	void **items;
	char **names;	
} list_obj;

void list_make(list_obj *listo, int size, char growable);
int list_add_item(list_obj *listo, void *item, char *name);
char* list_print_items(list_obj *listo);
void* list_get_name(list_obj *listo, char *name);
void* list_get_index(list_obj *listo, int indx);
void* list_get_item(list_obj *listo, void *item_to_find);
int list_find(list_obj *listo, char *name_to_find);
void list_delete_index(list_obj *listo, int indx);
void list_delete_name(list_obj *listo, char *name);
void list_delete_item(list_obj *listo, void *item);
void list_delete_all(list_obj *listo);
void list_print_list(list_obj *listo);
void list_free(list_obj *listo);

void test_list();
#endif
