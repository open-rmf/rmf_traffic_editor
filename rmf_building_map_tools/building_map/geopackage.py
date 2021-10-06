import sqlite3


class GeoPackage:
    def __init__(self, filename):
        self.filename = filename
        self.conn = None

    def __enter__(self):
        self.conn = sqlite3.connect(self.filename)
        return self

    def __exit__(self, type, value, tb):
        self.conn.commit()
        self.conn.close()
        self.conn = None

    def set_metadata(self, metadata):
        self.create_metadata_tables()
        cursor = self.conn.cursor()

        # remove our previous metadata document (if any)
        cursor.execute('DELETE FROM gpkg_metadata '
                       'WHERE md_scope = "rmfTrafficMap";')

        cursor.execute(
            'INSERT INTO gpkg_metadata('
            'md_scope, md_standard_uri, mime_type, metadata) '
            'VALUES(?, ?, ?, ?);',
            (
                'rmfTrafficMap',
                'http://open-rmf.org',
                'application/json',
                metadata
            ))

    def get_metadata(self):
        cursor = self.conn.cursor()
        rows = cursor.execute("SELECT metadata FROM gpkg_metadata "
                              "WHERE md_scope = 'rmfTrafficMap'")
        if not rows:
            return ''
        for row in rows:
            return row[0]

    def create_metadata_tables(self):
        cursor = self.conn.cursor()
        cursor.execute("SELECT name FROM sqlite_master "
                       "WHERE type='table' AND name='gpkg_metadata'")
        if len(cursor.fetchall()) > 0:
            return

        cursor.execute('''
            CREATE TABLE gpkg_metadata(
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                md_scope TEXT NOT NULL,
                md_standard_uri TEXT NOT NULL,
                mime_type TEXT NOT NULL,
                metadata TEXT NOT NULL
            );''')

        cursor.execute('''
            CREATE TABLE gpkg_metadata_reference(
                reference_scope TEXT NOT NULL,
                table_name TEXT,
                column_name TEXT,
                row_id_value INTEGER,
                timestamp DATETIME NOT NULL,
                md_file_id INTEGER NOT NULL,
                md_parent_id INTEGER,
                CONSTRAINT crmr_mfi_fk
                    FOREIGN KEY (md_file_id)
                    REFERENCES gpkg_metadata(id),
                CONSTRAINT crmr_mpi_fk
                    FOREIGN KEY (md_parent_id)
                    REFERENCES gpkg_metadata(id)
            );''')

        cursor.execute('''
            INSERT INTO gpkg_extensions
            (
                table_name,
                column_name,
                extension_name,
                definition,
                scope
            )
            VALUES
            (
                'gpkg_metadata',
                NULL,
                'gpkg_metadata',
                'http://www.geopackage.org/spec121/#extension_metadata',
                'read-write'
            );''')

        cursor.execute('''
            INSERT INTO gpkg_extensions
            (
                table_name,
                column_name,
                extension_name,
                definition,
                scope
            )
            VALUES
            (
                'gpkg_metadata_reference',
                NULL,
                'gpkg_metadata',
                'http://www.geopackage.org/spec121/#extension_metadata',
                'read-write'
            );''')
